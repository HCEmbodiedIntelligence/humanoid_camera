#!/usr/bin/env python3
"""Collect bounded, read-only host/ROS evidence without subscribing to images.

Works directly from a checkout with the Python standard library. No camera SDK
device is opened, no ROS parameters are set, and no existing process is stopped.
"""
import argparse
from contextlib import suppress
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import time
import uuid
import zipfile


ENV_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
            'ROS_DISCOVERY_SERVER', 'ROS_LOG_DIR', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
            'RMW_FASTRTPS_PUBLICATION_MODE')
PROCESS_NAMES = {'realsense2_camera_node', 'timestamp_adapter.py', 'gain_controller.py',
                 'configurator_launcher.py'}
OUTPUT_LIMIT = 1024 * 1024


def report_window(report):
    """Infer the original sampling wall time, not this collector's current time."""
    starts = []
    duration = float(report['duration_sec'])
    if not math.isfinite(duration) or not 0 < duration <= 86400:
        raise ValueError('报告 duration_sec 无效')
    for camera in report['cameras']:
        for samples in camera.get('raw_metadata_samples', {}).values():
            first = samples.get('first') or {}
            if 'receive_ns' in first and 'elapsed_sec' in first:
                start = int(first['receive_ns']) / 1e9 - float(first['elapsed_sec'])
                if math.isfinite(start) and start > 0:
                    starts.append(start)
    if not starts:
        raise ValueError('报告缺少接收时间，无法定位当时的日志；请使用新版验收报告')
    return min(starts), max(starts) + duration


def run_command(argv, env, timeout=8, limit=OUTPUT_LIMIT):
    """Bound output while draining it; kill only our own command on timeout."""
    started = time.monotonic()
    result = {'argv': argv, 'timeout_sec': timeout}
    chunks, total = [], 0
    try:
        with subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              env=env, start_new_session=True) as process:
            with selectors.DefaultSelector() as reader:
                reader.register(process.stdout, selectors.EVENT_READ)
                try:
                    while reader.get_map():
                        if time.monotonic() - started >= timeout:
                            result['timed_out'] = True
                            with suppress(ProcessLookupError):
                                os.killpg(process.pid, signal.SIGKILL)
                            break
                        for key, _ in reader.select(.1):
                            data = os.read(key.fileobj.fileno(), 65536)
                            if not data:
                                reader.unregister(key.fileobj)
                                continue
                            if total < limit:
                                chunks.append(data[:limit - total])
                            total += len(data)
                    if not result.get('timed_out'):
                        try:
                            process.wait(timeout=max(.01, timeout - (time.monotonic() - started)))
                        except subprocess.TimeoutExpired:
                            result['timed_out'] = True
                finally:
                    if process.poll() is None:
                        # This private process group belongs to this invocation.
                        with suppress(ProcessLookupError):
                            os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            result['returncode'] = process.returncode
    except OSError as error:
        result['error'] = str(error)
    result.update(duration_sec=time.monotonic() - started, bytes_read=total,
                  output_truncated=total > limit)
    return result, b''.join(chunks).decode('utf-8', errors='replace')


def read_file(path, limit=OUTPUT_LIMIT):
    try:
        with Path(path).open('rb') as stream:
            data = stream.read(limit + 1)
        return {'text': data[:limit].decode('utf-8', errors='replace'),
                'truncated': len(data) > limit}
    except OSError as error:
        return {'error': str(error)}


def parse_snmp(text):
    result = {}
    lines = text.splitlines()
    for header, values in zip(lines[::2], lines[1::2]):
        keys, numbers = header.split(), values.split()
        if keys and numbers and keys[0] == numbers[0] and len(keys) == len(numbers):
            for key, value in zip(keys[1:], numbers[1:]):
                try:
                    result[keys[0].rstrip(':') + '.' + key] = int(value)
                except ValueError:
                    pass
    return result


def counters_delta(before, after):
    """Counter reset must not be presented as zero packet loss."""
    return {key: {'delta': after[key] - value if after[key] >= value else None,
                  'reset': after[key] < value}
            for key, value in before.items() if key in after}


def host_snapshot():
    return {'wall_time_ns': time.time_ns(), 'monotonic_sec': time.monotonic(),
            'files': {path: read_file(path) for path in
                      ('/proc/net/snmp', '/proc/net/netstat', '/proc/net/dev',
                       '/proc/stat', '/proc/loadavg', '/proc/meminfo',
                       '/proc/pressure/cpu', '/proc/pressure/io', '/proc/pressure/memory')}}


def process_snapshot(proc=Path('/proc')):
    records, log_paths = [], set()
    for directory in proc.glob('[0-9]*'):
        argv = []
        try:
            argv = (directory / 'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
            if not any(Path(arg).name in PROCESS_NAMES for arg in argv[:3]):
                continue
            environ = {}
            for item in (directory / 'environ').read_bytes().decode(errors='replace').split('\0'):
                key, _, value = item.partition('=')
                if key in ENV_KEYS:
                    environ[key] = value
            item = {'pid': int(directory.name), 'argv': argv, 'environment': environ,
                    'stat': read_file(directory / 'stat'), 'status': read_file(directory / 'status')}
            for fd in ('1', '2'):
                try:
                    path = (directory / 'fd' / fd).resolve(strict=True)
                    if path.is_file() and path.suffix in ('.log', '.txt'):
                        log_paths.add(path)
                except OSError:
                    pass
            maps = read_file(directory / 'maps')
            item['loaded_camera_transport_libraries'] = sorted(set(
                line.split()[-1] for line in maps.get('text', '').splitlines()
                if re.search(r'lib(realsense|usb|fastrtps|fastdds|rmw)', line)))
            if 'error' in maps:
                item['maps_error'] = maps['error']
            records.append(item)
        except OSError as error:
            # Ignore unrelated inaccessible processes; expose errors for targets.
            if any(Path(arg).name in PROCESS_NAMES for arg in argv[:3]):
                records.append({'pid': int(directory.name), 'error': str(error)})
    return records, log_paths


def collect_logs(roots, direct_paths, since, destination, max_files=12, limit=OUTPUT_LIMIT):
    """Keep bounded tails; report paths, truncation and permission failures."""
    candidates, errors = set(direct_paths), []
    for root in roots:
        try:
            if not root.is_dir():
                errors.append({'path': str(root), 'error': '日志目录不存在或不可访问'})
                continue
            candidates.update(root.glob('*.log'))
            candidates.update(root.glob('*/*.log'))
        except OSError as error:
            errors.append({'path': str(root), 'error': str(error)})
    selected = []
    for path in candidates:
        try:
            stat = path.stat()
            if path.is_file() and (path in direct_paths or stat.st_mtime >= since):
                # Active camera stdout takes precedence over unrelated new logs.
                selected.append((path in direct_paths, stat.st_mtime, str(path), path))
        except OSError as error:
            errors.append({'path': str(path), 'error': str(error)})
    selected.sort(reverse=True)
    records = []
    destination.mkdir()
    for index, (_, _, _, path) in enumerate(selected[:max_files]):
        try:
            with path.open('rb') as stream:
                size = os.fstat(stream.fileno()).st_size
                offset = max(0, size - limit)
                stream.seek(offset)
                data = stream.read(min(size, limit))
            target = destination / f'{index:02d}-{path.name}'
            target.write_bytes(data)
            records.append({'source': str(path), 'file': 'logs/' + target.name,
                            'source_bytes': size, 'offset_bytes': offset,
                            'tail_only': offset > 0, 'sha256': hashlib.sha256(data).hexdigest()})
        except OSError as error:
            errors.append({'path': str(path), 'error': str(error)})
    return {'files': records, 'errors': errors, 'omitted_by_file_limit': max(0, len(selected) - max_files),
            'max_files': max_files, 'max_bytes_per_file': limit,
            'selection': 'active camera stdout first, then recently modified logs; bounded tails may omit the original event'}


def command_plan(node, start, end):
    return [
        ('packages', ['dpkg-query', '-W', 'ros-humble-realsense2-*', 'ros-humble-librealsense2*', 'librealsense2*']),
        ('usb_tree', ['lsusb', '-t']),
        ('kernel_test_window', ['journalctl', '-k', '--since=@' + str(int(start - 60)),
                                '--until=@' + str(int(end + 60)), '--no-pager', '-o', 'short-unix']),
        ('kernel_current', ['journalctl', '-k', '--since=-5min', '--no-pager', '-o', 'short-unix']),
        ('nodes', ['ros2', 'node', 'list', '--no-daemon']),
        ('camera_node', ['ros2', 'node', 'info', node, '--no-daemon']),
        ('recorder_node', ['ros2', 'node', 'info', '/humanoid_manager_observer_recorder', '--no-daemon']),
        ('parameters', ['ros2', 'param', 'dump', node]),
        ('device_info', ['ros2', 'service', 'call', node + '/device_info',
                         'realsense2_camera_msgs/srv/DeviceInfo', '{}']),
    ] + [(suffix.replace('/', '_'), ['ros2', 'topic', 'info', node + '/' + suffix,
                                     '--verbose', '--no-daemon'])
         for suffix in ('color/image_raw', 'depth/image_rect_raw', 'color/metadata', 'depth/metadata')]


def main():
    parser = argparse.ArgumentParser(description='只读收集相机缺帧诊断：不订阅图像、不打开相机设备、不改参数。')
    parser.add_argument('--report', type=Path, required=True, help='已完成验收的 report.json 或所在目录')
    parser.add_argument('--camera', default='camera_hand')
    parser.add_argument('--node', help='完整驱动节点名；默认 /<camera>/camera')
    parser.add_argument('--domain-id', type=int, help='默认取验收报告 domain_id')
    parser.add_argument('--log-root', type=Path, action='append', default=[], help='额外 launch/管理器日志目录，可重复')
    parser.add_argument('--output', type=Path, default=Path('./camera_diagnostics'))
    args = parser.parse_args()
    report_path = args.report.expanduser()
    if report_path.is_dir():
        report_path /= 'report.json'
    try:
        if report_path.stat().st_size > 20 * OUTPUT_LIMIT:
            raise ValueError('报告超过 20 MiB')
        report_bytes = report_path.read_bytes()
        report = json.loads(report_bytes)
        start, end = report_window(report)
        if args.camera not in {camera['id'] for camera in report['cameras']}:
            raise ValueError('报告中没有所选相机')
        domain = int(args.domain_id if args.domain_id is not None else report['domain_id'])
        node = args.node or '/' + args.camera + '/camera'
        if not 0 <= domain <= 232 or not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z_0-9]*)+', node):
            raise ValueError('ROS domain_id 或节点名称无效')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    directory = args.output.expanduser() / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6])
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'source_report.json').write_bytes(report_bytes)
    env = {**os.environ, 'ROS_DOMAIN_ID': str(domain)}
    before = host_snapshot()
    processes_before, active_logs = process_snapshot()
    manifest = {'schema_version': 1, 'kind': 'read_only_camera_diagnostics', 'camera': args.camera,
                'node': node, 'domain_id': domain, 'source_report': str(report_path),
                'source_report_sha256': hashlib.sha256(report_bytes).hexdigest(),
                'test_wall_window_sec': [start, end], 'environment': {key: env[key] for key in ENV_KEYS if key in env},
                'commands': [], 'limitations': [
                    '当前 ROS/USB/进程状态不等于历史采样时状态；原测试时间段单独用于内核日志查询。',
                    '无内核日志或权限不足不能排除 USB 故障；日志是有长度上限的尾部，可能缺少原事件。',
                    '主机网络计数涵盖全部进程，不是相机专属统计；未订阅图像，未测量当前相机帧率。']}
    print('诊断目录：' + str(directory.resolve()), flush=True)
    interrupted = False
    try:
        for label, argv in command_plan(node, start, end):
            print('读取：' + label, flush=True)
            result, output = run_command(argv, env)
            filename = label + '.txt'
            (directory / filename).write_text(output)
            manifest['commands'].append({'file': filename, **result})
        remaining = 10 - (time.monotonic() - before['monotonic_sec'])
        if remaining > 0:
            time.sleep(remaining)
    except KeyboardInterrupt:
        interrupted = True
    after = host_snapshot()
    processes_after, more_logs = process_snapshot()
    knobs = ['/proc/sys/net/core/' + name for name in ('rmem_max', 'rmem_default', 'wmem_max', 'wmem_default')]
    knobs += ['/proc/sys/net/ipv4/' + name for name in ('ipfrag_high_thresh', 'ipfrag_time')]
    manifest['host'] = {'before': before, 'after': after,
                        'elapsed_sec': after['monotonic_sec'] - before['monotonic_sec'],
                        'network_settings': {path: read_file(path) for path in knobs}}
    manifest['host']['snmp_delta'] = counters_delta(
        parse_snmp(before['files']['/proc/net/snmp'].get('text', '')),
        parse_snmp(after['files']['/proc/net/snmp'].get('text', '')))
    manifest['processes'] = {'before': processes_before, 'after': processes_after}
    roots = [Path(os.environ.get('ROS_LOG_DIR', str(Path.home() / '.ros/log'))),
             Path.home() / '.local/share/humanoid-manager/runtime_logs'] + [p.expanduser() for p in args.log_root]
    manifest['logs'] = collect_logs(roots, active_logs | more_logs, start - 60, directory / 'logs')
    manifest['interrupted'] = interrupted
    (directory / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    (directory / 'README.txt').write_text(
        '本包用于定位缺帧，不给出相机通过/失败结论。\n'
        'source_report.json 是之前的验收结果；kernel_test_window.txt 查询那个时间段的内核日志。\n'
        '其他 ROS、USB、主机计数和进程信息在本次诊断运行时读取，两次时间窗口不要混用。\n'
        'manifest.json 记录每条命令的退出码、超时、输出截断、日志来源及读取错误。\n'
        '日志有上限；无输出不证明没有故障。脚本不订阅图像、不修改参数、不停止相机。\n')
    archive = directory.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as target:
        for path in sorted(directory.rglob('*')):
            if path.is_file():
                target.write(path, path.relative_to(directory))
    missing = sum(item.get('returncode') != 0 or item.get('timed_out', False) for item in manifest['commands'])
    print(f'已保存诊断包：{archive.resolve()}\n命令失败或超时 {missing} 项；详细限制见 manifest.json。')
    return 130 if interrupted else 0


if __name__ == '__main__':
    raise SystemExit(main())
