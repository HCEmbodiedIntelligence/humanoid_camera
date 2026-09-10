#!/usr/bin/env python3
"""Manually sample camera metadata and read parameters, then write an acceptance report."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from humanoid_camera.health_check import CameraCheck, expected_parameters
from humanoid_camera.health_report import assemble, console_report, write_report, create_report_directory, bundle_report


def stamp(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def run(cameras, args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from realsense2_camera_msgs.msg import Metadata, RGBD
    from std_msgs.msg import String
    from humanoid_camera.transport import CaptureSubscription
    evidence = None
    if getattr(args, 'save_evidence', False):
        from humanoid_camera.evidence import EvidenceRecorder
        evidence = EvidenceRecorder(args.report_directory, interval=args.evidence_interval, max_pairs=args.evidence_max_pairs,
                                    depth_min_m=args.depth_preview_min_m, depth_max_m=args.depth_preview_max_m)
    rclpy.init(args=[], domain_id=args.domain_id)
    node = rclpy.create_node('humanoid_manual_camera_check_' + str(os.getpid()))
    checks = [CameraCheck(camera, verify_images=args.verify_images, exercise_auto=args.exercise_auto,
                          max_age_ms=args.max_age_ms,
                          require_equal_exposure=getattr(args, 'require_equal_exposure', False)) for camera in cameras]
    parameters, clients, futures = {}, {}, {}
    capture_subscriptions=[]
    next_transport_refresh=time.monotonic()+1.
    started = time.monotonic() + args.warmup
    deadline = started + args.duration
    last_progress = started
    interrupted = False
    runtime_error = None
    sampled_duration = args.duration
    try:
        for check in checks:
            camera, ident = check.camera, check.camera['id']
            prefix = '/' + camera['namespace'] + '/' + camera['camera_name']
            names = list(expected_parameters(camera)) + ['use_sim_time']
            clients[ident] = (node.create_client(GetParameters, prefix + '/get_parameters'), names)

            def metadata(message, check=check, stream=None):
                elapsed = time.monotonic() - started
                if elapsed < 0: return
                try:
                    raw = json.loads(message.json_data)
                    if not isinstance(raw, dict): raise ValueError('metadata must be an object')
                    if stream:
                        check.raw(stream, raw, stamp(message.header), node.get_clock().now().nanoseconds, elapsed)
                    else:
                        check.normalized(raw, stamp(message.header), elapsed)
                        if evidence:
                            evidence.metadata_received(check.camera, raw, stamp(message.header))
                except (ValueError, TypeError, KeyError, OverflowError):
                    check.fail((stream or 'normalized') + ':无效metadata')

            for stream, suffix in [('rgb', 'color/metadata'), ('depth', 'depth/metadata')]:
                capture_subscriptions.append(CaptureSubscription(node, Metadata, prefix + '/' + suffix,
                    lambda msg, stream=stream, cb=metadata: cb(msg, stream=stream), depth=30))
            capture_subscriptions.append(CaptureSubscription(node, Metadata, camera['metadata_topic'], metadata, depth=30))
            def diagnostics(message, check=check):
                if time.monotonic() < started:
                    return
                try:
                    check.pipeline(json.loads(message.data))
                except (ValueError, TypeError, KeyError, OverflowError, AttributeError):
                    pass  # Optional diagnostic data never changes acceptance criteria.
            node.create_subscription(String, '/' + camera['namespace'] + '/normalized/diagnostics',
                                     diagnostics, qos_profile_sensor_data)
            if args.verify_images:
                def image(message, check=check):
                    elapsed = time.monotonic() - started
                    if elapsed >= 0:
                        check.image(stamp(message.header), stamp(message.rgb.header), stamp(message.depth.header), elapsed)
                        if evidence:
                            evidence.image_received(check.camera, message, elapsed)
                capture_subscriptions.append(CaptureSubscription(node, RGBD, camera['rgbd_topic'], image))
        print(f'已选择 {len(checks)} 台相机；等待 {args.warmup:g}s 后采样 {args.duration:g}s。仅订阅话题、读取参数。', flush=True)
        if getattr(args, 'require_equal_exposure', False):
            print('要求每对 RGB/深度曝光换算到μs后完全相等；曝光中点差按配置阈值独立检查。', flush=True)
        if args.exercise_auto:
            print('自动补偿测试：请在采样期间手动改变光照，再恢复，观察曝光或增益是否变化。', flush=True)
        while time.monotonic() < deadline:
            for ident, (client, names) in clients.items():
                if ident not in futures and client.service_is_ready():
                    request = GetParameters.Request(); request.names = names
                    futures[ident] = client.call_async(request)
                future = futures.get(ident)
                if future is not None and future.done() and ident not in parameters:
                    try:
                        values = future.result().values
                        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
                        parameters[ident] = {name: getattr(value, fields[value.type]) if value.type in fields else None
                                             for name, value in zip(names, values)}
                    except Exception:
                        parameters[ident] = None
            rclpy.spin_once(node, timeout_sec=min(.1, max(0, deadline - time.monotonic())))
            now = time.monotonic()
            if now>=next_transport_refresh:
                for subscription in capture_subscriptions:subscription.refresh()
                next_transport_refresh=now+1.
            if evidence:
                evidence.flush(now - started)
            if now >= last_progress + 5:
                last_progress = now
                print('采样 {:.0f}/{:.0f}s：{}'.format(now - started, args.duration, ' | '.join(
                    f"{c.camera['id']} RGB={c.counts['rgb']} 深度={c.counts['depth']} 配对校验={c.counts['verified_pairs']}"
                    for c in checks)), flush=True)
    except KeyboardInterrupt:
        interrupted = True
        sampled_duration = max(.001, min(args.duration, time.monotonic() - started))
    except Exception as error:
        runtime_error = str(error)
        sampled_duration = max(.001, min(args.duration, time.monotonic() - started))
    finally:
        evidence_result = evidence.close() if evidence else None
        node.destroy_node(); rclpy.try_shutdown()
    report = assemble(checks, sampled_duration, parameters, verify_images=args.verify_images)
    report['interrupted'] = interrupted
    report['receiver_qos']={subscription.topic:subscription.state() for subscription in capture_subscriptions}
    if interrupted:
        for camera in report['cameras']:
            camera['unknowns']['采样被中断，未完成预定时长'] = 1
            if camera['status'] == 'PASS':
                camera['status'] = 'UNKNOWN'
        if report['status'] == 'PASS':
            report['status'] = 'UNKNOWN'
    if runtime_error:
        report['runtime_error'] = runtime_error
        report['status'] = 'FAIL'
    if evidence_result:
        report['evidence'] = evidence_result
        if not evidence_result['saved'] or evidence_result['errors'] or any(item['metadata_status'] != 'matched' for item in evidence_result['saved']):
            if report['status'] == 'PASS':
                report['status'] = 'UNKNOWN'
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path, help='已部署的 cameras.yaml')
    parser.add_argument('--camera', action='append', default=[], help='只验收指定 ID，可重复；默认全部启用的 RealSense')
    parser.add_argument('--duration', type=float, default=30., help='采样秒数，默认30')
    parser.add_argument('--warmup', type=float, default=3., help='开始采样前等待秒数')
    parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--max-age-ms', type=float, default=500., help='采集到本检查器接收的最大间隔，非时钟精度')
    parser.add_argument('--verify-images', action='store_true', help='额外订阅 RGBD 图像，核对 RGB/深度 Header，增加图像传输开销')
    parser.add_argument('--exercise-auto', action='store_true', help='要求观察到自动曝光或增益变化；请手动改变光照')
    parser.add_argument('--require-equal-exposure', action='store_true',
                        help='要求每对 RGB/深度的实际曝光时长相等（差值0 μs）；独立于曝光中点对齐检查')
    parser.add_argument('--output', type=Path, default=Path('camera_checks'))
    parser.add_argument('--save-evidence', action='store_true', help='保存真实RGB/深度PNG及逐帧证据，并打包ZIP；同时启用图像验证')
    parser.add_argument('--evidence-interval', type=float, default=10., help='每台相机抽样照片间隔秒数，默认10')
    parser.add_argument('--evidence-max-pairs', type=int, default=6, help='每台最多保存多少对照片，默认6')
    parser.add_argument('--depth-preview-min-m', type=float, default=.2, help='深度伪彩色显示下限（米），默认0.2；不影响采集数据')
    parser.add_argument('--depth-preview-max-m', type=float, default=2., help='深度伪彩色显示上限（米），默认2.0；不影响采集数据')
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not 1 <= args.duration <= 3600 or
            not math.isfinite(args.warmup) or not 0 <= args.warmup <= 60 or
            not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0 or not 0 <= args.domain_id <= 232 or
            not math.isfinite(args.evidence_interval) or args.evidence_interval < 1 or not 1 <= args.evidence_max_pairs <= 120):
        parser.error('无效采样时长、等待时间、延迟阈值或 domain_id')
    if not (math.isfinite(args.depth_preview_min_m) and math.isfinite(args.depth_preview_max_m)
            and 0 <= args.depth_preview_min_m < args.depth_preview_max_m <= 65.535):
        parser.error('深度预览范围必须满足 0 ≤ 最小值 < 最大值 ≤ 65.535 m')
    try:
        import yaml
        from humanoid_camera.configuration import validate_cameras
        from humanoid_manager.plugin_metadata import resolved_document
        document = yaml.safe_load(args.config.expanduser().read_text())
        if not isinstance(document, dict) or document.get('schema_version') != 1:
            raise ValueError('配置必须包含 schema_version: 1 和 cameras 列表')
        cameras = [resolved_document(c, c) for c in validate_cameras(document['cameras']) if c['enabled'] and c['backend'] == 'realsense']
        if set(args.camera) - {c['id'] for c in cameras}:
            raise ValueError('指定相机不存在、已禁用或不是 RealSense')
        if args.camera: cameras = [c for c in cameras if c['id'] in args.camera]
        if not cameras: raise ValueError('没有可验收的已启用 RealSense 相机')
        if args.save_evidence:
            # Fail before sampling if PNG support is not installed.
            try:
                from humanoid_camera.evidence import EvidenceRecorder
            except ImportError as error:
                raise RuntimeError('保存照片需要 Pillow/NumPy，请安装 python3-pil 和 python3-numpy') from error
            args.verify_images = True
            args.report_directory = create_report_directory(args.output)
            print('证据保存目录：' + str(args.report_directory.resolve()), flush=True)
        report = run(cameras, args)
        report['config_file'] = str(args.config.expanduser().resolve())
        report['domain_id'] = args.domain_id
        report['exercise_auto'] = args.exercise_auto
        report['require_equal_exposure'] = args.require_equal_exposure
        destination = write_report(report, args.output, destination=getattr(args, 'report_directory', None))
        print(console_report(report))
        print('报告：' + str(destination.resolve() / 'report.html'))
        if args.save_evidence:
            print(f"已保存 {len(report['evidence']['saved'])} 对照片证据；写入错误 {len(report['evidence']['errors'])} 项。")
            print('汇报压缩包：' + str(bundle_report(destination).resolve()))
        if report.get('interrupted'):
            print('采样被中断；已保留照片和部分结果，不能作为完整验收。')
            return 130
        return {'PASS': 0, 'FAIL': 1, 'UNKNOWN': 2}[report['status']]
    except KeyboardInterrupt:
        print('验收已中断，未生成通过结论。', file=sys.stderr); return 130
    except Exception as error:
        print('无法完成验收：' + str(error), file=sys.stderr); return 2


if __name__ == '__main__':
    raise SystemExit(main())
