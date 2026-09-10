"""Diagnostic collection must stay bounded and distinguish historical evidence."""
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest

spec = importlib.util.spec_from_file_location(
    'collect_camera_diagnostics', Path(__file__).resolve().parents[1] / 'scripts/collect_camera_diagnostics.py')
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


def test_historical_log_window_uses_reception_minus_sampling_elapsed():
    report = {'duration_sec': 60, 'cameras': [{'raw_metadata_samples': {
        'rgb': {'first': {'receive_ns': 1789018897866193658, 'elapsed_sec': .020083569}},
        'depth': {'first': {'receive_ns': 1789018902797981775, 'elapsed_sec': 4.951898767}}
    }}]}
    start, end = collector.report_window(report)
    assert start == pytest.approx(1789018897.846, abs=.001)
    assert end - start == pytest.approx(60, abs=.001)
    with pytest.raises(ValueError):
        collector.report_window({'duration_sec': 60, 'cameras': [{}]})


def test_commands_do_not_start_camera_change_parameters_or_subscribe_to_images():
    commands = dict(collector.command_plan('/camera_hand/camera', 100, 160))
    assert '--since=@40' in commands['kernel_test_window']
    assert '--until=@220' in commands['kernel_test_window']
    for argv in commands.values():
        assert not any(token in argv for token in ('sudo', 'set', 'launch', 'run', 'echo', 'hz', 'bw'))
    assert commands['device_info'][-2:] == ['realsense2_camera_msgs/srv/DeviceInfo', '{}']


def test_command_output_limit_missing_binary_and_timeout():
    result, output = collector.run_command([sys.executable, '-c', "print('x' * 10000)"], os.environ, limit=64)
    assert result['returncode'] == 0 and result['output_truncated']
    assert len(output) == 64
    result, _ = collector.run_command(['/nonexistent/camera-diagnostic-tool'], os.environ)
    assert 'error' in result
    result, _ = collector.run_command([sys.executable, '-c', 'import time; time.sleep(30)'], os.environ, timeout=.15)
    assert result['timed_out'] and result['returncode'] < 0 and result['duration_sec'] < 3


def test_snmp_counts_expose_resets():
    before = collector.parse_snmp('Udp: InErrors RcvbufErrors\nUdp: 100 90\nIp: ReasmFails\nIp: 15\n')
    after = collector.parse_snmp('Udp: InErrors RcvbufErrors\nUdp: 103 93\nIp: ReasmFails\nIp: 2\n')
    delta = collector.counters_delta(before, after)
    assert delta['Udp.RcvbufErrors'] == {'delta': 3, 'reset': False}
    assert delta['Ip.ReasmFails'] == {'delta': None, 'reset': True}


def test_log_tail_is_bounded_and_active_driver_log_prioritized(tmp_path):
    root = tmp_path / 'runtime_logs'; root.mkdir()
    active = root / 'old-camera.log'; active.write_bytes(b'0123456789')
    os.utime(active, (10, 10))
    (root / 'new-observer.log').write_text('unrelated')
    result = collector.collect_logs([root], {active}, 100, tmp_path / 'collected', max_files=1, limit=4)
    assert result['omitted_by_file_limit'] == 1
    record = result['files'][0]
    assert record['source'] == str(active)
    assert record['tail_only'] and record['offset_bytes'] == 6
    assert (tmp_path / 'collected' / Path(record['file']).name).read_bytes() == b'6789'


def test_process_collection_whitelists_environment_and_libraries(tmp_path):
    proc = tmp_path / 'proc'; target = proc / '123'; target.mkdir(parents=True)
    (target / 'cmdline').write_bytes(b'/opt/ros/lib/realsense2_camera/realsense2_camera_node\0')
    (target / 'environ').write_bytes(b'ROS_DOMAIN_ID=14\0SECRET_TOKEN=do-not-collect\0')
    (target / 'stat').write_text('cpu counters')
    (target / 'status').write_text('VmRSS: 123')
    (target / 'maps').write_text('000 r-x 0 0 0 /opt/lib/librealsense2.so\n000 r-x 0 0 0 /private/other\n')
    (target / 'fd').mkdir()
    logfile = tmp_path / 'robot.log'; logfile.write_text('driver output')
    (target / 'fd/1').symlink_to(logfile)
    records, paths = collector.process_snapshot(proc)
    assert records[0]['environment'] == {'ROS_DOMAIN_ID': '14'}
    assert records[0]['loaded_camera_transport_libraries'] == ['/opt/lib/librealsense2.so']
    assert paths == {logfile}
    assert 'do-not-collect' not in json.dumps(records)
