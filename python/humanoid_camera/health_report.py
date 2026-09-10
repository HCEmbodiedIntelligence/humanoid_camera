"""Standalone, human-readable reports for an explicitly requested acceptance run."""
import csv
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import uuid
import zipfile


def value(metric, key='max'):
    result = (metric or {}).get(key)
    return '—' if result is None else f'{result:.3f}'


def console_report(report):
    lines = [f"相机手动验收：{report['status']}  |  来源：{report['data_origin']}  |  {report['duration_sec']:.1f}s",
             '相机 | 状态 | RGB/深度 Hz | RGB/深度最大曝光 us | RGB-D曝光时长差 max us | 中点差 p95/max ms | 换算残差 max ns']
    if report.get('runtime_error'):
        lines.append('采样错误：' + report['runtime_error'])
    for camera in report['cameras']:
        metrics, fps = camera['metrics'], camera['fps']
        skew = metrics.get('rgb_depth_midpoint_skew_ms')
        lines.append(f"{camera['id']} | {camera['status']} | {fps['rgb']:.1f}/{fps['depth']:.1f} | "
                     f"{value(metrics.get('rgb_exposure_us'))}/{value(metrics.get('depth_exposure_us'))} | "
                     f"{value(metrics.get('rgb_depth_exposure_difference_us'))} | "
                     f"{value(skew, 'p95_recent')}/{value(skew)} | {value(metrics.get('mapping_residual_ns'))}")
        lines.append('  曝光时长一致性：' + ('要求每对差值为0 μs' if camera.get('require_equal_exposure')
                                         else '仅报告差值，未要求相等'))
        scales = camera.get('metadata_exposure_scale_us', {})
        if scales.get('rgb') == 100:
            lines.append('  D435 RGB曝光元数据：原始值×100换算为μs；时间戳字段不作此换算。')
        for stream, samples in camera.get('raw_metadata_samples', {}).items():
            first, last = samples['first'], samples['last']
            lines.append(f"  {stream}元数据：首帧在采样开始后{first['elapsed_sec']:.3f}s；"
                         f"末帧原始曝光={last['raw'].get('actual_exposure')}；"
                         f"sensor−hw偏移 max={value(metrics.get(stream + '_sensor_minus_hw_us'))} μs")
        for key, label in [('failures', '失败'), ('unknowns', '未确认')]:
            for message, count in camera[key].items():
                lines.append(f'  [{label}] {message}: {count}')
    lines.append('PASS 仅表示本次观测项满足阈值；SDK 映射的绝对误差、物理同步精度未独立测量。')
    return '\n'.join(lines)


def create_report_directory(root):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = Path(root).expanduser() / (stamp + '-' + uuid.uuid4().hex[:6])
    path.mkdir(parents=True)
    return path


def bundle_report(path):
    target = path.with_suffix('.zip')
    with zipfile.ZipFile(target, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for source in sorted(path.rglob('*')):
            if source.is_file():
                archive.write(source, Path(path.name) / source.relative_to(path))
    return target


def write_report(report, root, *, destination=None):
    path = Path(destination) if destination is not None else create_report_directory(root)
    (path / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    (path / 'summary.txt').write_text(console_report(report) + '\n', encoding='utf-8')
    escape = lambda item: html.escape(str(item))
    parts = ['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>相机手动验收</title>',
        '<style>body{font:16px/1.6 system-ui,sans-serif;max-width:1200px;margin:32px auto;padding:0 20px;color:#243547;background:#f4f7fa}table{border-collapse:collapse;width:100%;margin:14px 0;background:white}th,td{border:1px solid #ccd6de;padding:8px;text-align:left}th{background:#e4ecf3}section{margin:24px 0;padding:20px;background:white;border-radius:8px}.PASS{color:#08733c}.FAIL{color:#b12d26}.UNKNOWN{color:#865600}pre{white-space:pre-wrap;overflow-wrap:anywhere}small{color:#506174}</style>',
        f"<h1>相机手动验收 <span class='{report['status']}'>{report['status']}</span></h1>",
        f"<p>数据来源：{escape(report['data_origin'])} · 采样 {report['duration_sec']:.1f} 秒</p>",
        '<p>这里核查本次采样的曝光与时间戳行为。换算残差是软件一致性检查，不是物理时钟误差。绝对同步精度尚未独立测量。</p>']
    if report['data_origin'] != 'ROS 实际话题':
        parts.append('<p class="UNKNOWN"><strong>这是合成数据示例，不能用作真实设备验收记录。</strong></p>')
    if report.get('runtime_error'):
        parts.append('<p class="FAIL">采样错误：' + escape(report['runtime_error']) + '</p>')
    labels = {'rgb_exposure_us': 'RGB 实际曝光（μs）', 'depth_exposure_us': '深度实际曝光（μs）',
              'rgb_depth_exposure_difference_us': 'RGB/深度实际曝光时长差（μs）',
              'rgb_gain': 'RGB 增益', 'depth_gain': '深度增益',
              'rgb_depth_midpoint_skew_ms': 'RGB/深度曝光中点差（ms）',
              'mapping_residual_ns': '发布值与原始 metadata 换算残差（ns）',
              'rgb_receive_age_ms': 'RGB 采集到检查器接收间隔（ms）',
              'depth_receive_age_ms': '深度采集到检查器接收间隔（ms）'}
    for camera in report['cameras']:
        parts.append(f"<section><h2>{escape(camera['id'])} <span class='{camera['status']}'>{camera['status']}</span></h2>"
                     f"<p>型号 {escape(camera['device_type'])} · 配置序列号 {escape(camera['serial_no'])}</p>"
                     f"<p>观测帧率 RGB/深度/标准化：{camera['fps']['rgb']:.2f} / {camera['fps']['depth']:.2f} / {camera['fps']['normalized']:.2f} Hz</p>"
                     f"<p>阈值：实际曝光 ≤ {camera['limits']['exposure_us']} μs；中点差 ≤ {camera['limits']['midpoint_skew_ms']} ms。</p>")
        parts.append('<p>曝光时长一致性：' + ('要求每对实际曝光时长差为 0 μs。' if camera.get('require_equal_exposure')
                                            else '仅统计差值，未要求相等。') + '</p>')
        parts.append('<table><tr><th>观测项</th><th>样本数</th><th>最小</th><th>平均</th><th>P95</th><th>最大</th></tr>')
        for key, label in labels.items():
            metric = camera['metrics'].get(key, {})
            parts.append(f"<tr><td>{label}</td><td>{metric.get('samples', 0)}</td>" +
                         ''.join(f'<td>{value(metric, col)}</td>' for col in ('min', 'mean', 'p95_recent', 'max')) + '</tr>')
        parts.append('</table><small>P95 使用最近最多 2048 个样本；最小、平均、最大覆盖全部采样。CSV 保留最近最多 10000 对帧。</small>')
        for key, label, css in [('failures', '失败项', 'FAIL'), ('unknowns', '未确认项', 'UNKNOWN')]:
            if camera[key]:
                parts.append(f'<h3 class="{css}">{label}</h3><ul>')
                for message, count in camera[key].items(): parts.append(f'<li>{escape(message)}：{escape(count)}</li>')
                parts.append('</ul>')
        parts.append('<details><summary>计数及驱动参数回读</summary><pre>' + escape(json.dumps(
            {'counts': camera['counts'], 'parameters': camera['parameters'],
             'metadata_exposure_scale_us': camera.get('metadata_exposure_scale_us'),
             'raw_metadata_samples': camera.get('raw_metadata_samples')}, ensure_ascii=False, indent=2)) + '</pre></details></section>')
    evidence = report.get('evidence', {})
    if evidence.get('enabled'):
        parts.append('<section><h2>真实图像抽样证据</h2><p>RGB 为无损 PNG；深度数值 PNG 原样保存接收到的16位数值（包含当前驱动滤波结果）。'
                     '深度预览仅为显示拉伸，数值范围按原始像素计数，不表示已标定的米制距离。照片取自同一条 RGBD 消息；'
                     '曝光与增益仅在来源元数据逐帧匹配时展示。单张照片不能证明连续帧率或物理同步精度。</p>')
        if not evidence.get('saved'):
            parts.append('<p class="FAIL">未保存到照片证据，请查看接收计数和错误；本报告不能充当有图像证据的验收。</p>')
        for error in evidence.get('errors', []):
            parts.append('<p class="FAIL">' + escape(error) + '</p>')
        for item in evidence.get('saved', []):
            files, observations = item['files'], item['observations']
            parts.append(f"<h3>{escape(item['camera_id'])} · 样本 {item['sample_index']} · 采样后 {item['elapsed_sec']:.2f}s</h3>")
            parts.append('<p>元数据匹配：' + ('已匹配' if item['metadata_status'] == 'matched' else '未确认，曝光与增益未知') + '</p>')
            parts.append('<div style="display:flex;gap:16px;flex-wrap:wrap">')
            for key, label in [('rgb', 'RGB 原分辨率 PNG'), ('depth_preview', '深度显示预览')]:
                if key in files:
                    url = escape(files[key])
                    parts.append(f'<figure style="margin:0;max-width:46%"><a href="{url}"><img src="{url}" style="width:100%" alt="{label}"></a><figcaption>{label}</figcaption></figure>')
            parts.append('</div><table><tr><th>流</th><th>来源帧号</th><th>图像时间戳 ns</th><th>曝光 μs</th><th>增益</th></tr>')
            for stream in ('rgb', 'depth'):
                row = observations[stream]
                parts.append('<tr><td>' + stream + '</td>' + ''.join('<td>' + escape(row.get(key)) + '</td>'
                             for key in ('frame_number', 'header_ns', 'actual_exposure_us', 'gain')) + '</tr>')
            parts.append('</table><p>' + ' · '.join(f'<a href="{escape(files[key])}">{label}</a>' for key, label in
                         [('depth', '下载16位深度数值 PNG'), ('metadata', '查看此帧来源元数据和 SHA-256')]
                         if key in files) + '</p>')
        parts.append('</section>')
    parts.append('<p>图像验证：' + ('已启用' if report['verify_images'] else '未启用，仅验证元数据；完整验收请加 --verify-images') + '</p></html>')
    (path / 'report.html').write_text('\n'.join(parts), encoding='utf-8')
    with (path / 'samples.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=['camera_id', 'rgb_time_ns', 'depth_time_ns', 'skew_ms',
                                                   'rgb_exposure_us', 'depth_exposure_us', 'exposure_difference_us'])
        writer.writeheader()
        for camera in report['cameras']:
            writer.writerows({'camera_id': camera['id'], **row} for row in camera['trace'])
    return path


def assemble(checks, duration, parameters, *, verify_images, data_origin='ROS 实际话题'):
    cameras = [check.report(duration, parameters.get(check.camera['id'])) for check in checks]
    status = 'FAIL' if any(c['status'] == 'FAIL' for c in cameras) else 'UNKNOWN' if any(c['status'] == 'UNKNOWN' for c in cameras) else 'PASS'
    return {'schema_version': 1, 'status': status, 'data_origin': data_origin,
            'duration_sec': duration, 'verify_images': verify_images,
            'physical_clock_accuracy_verified': False, 'cameras': cameras}
