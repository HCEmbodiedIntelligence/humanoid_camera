"""Locate a missing input without changing frame association or timestamp math."""
from collections import OrderedDict
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as Obj

import pytest

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck
from humanoid_camera.health_report import console_report
from humanoid_camera.pipeline_diagnostics import PipelineDiagnostics, summarize_window


def test_rates_use_the_producer_window_and_restart_starts_a_new_window():
    stats = PipelineDiagnostics('front', 'first-process')
    first = stats.snapshot(pending_groups=0, pending_bytes=0, clock_epoch=0)
    last = copy.deepcopy(first)
    last.update(uptime_sec=first['uptime_sec'] + 10, published_pairs=50)
    last['received'].update(rgb=100, depth=300, rgb_meta=300, depth_meta=300)
    last['discarded_groups']['buffer_limit'] = 200
    last['missing_when_discarded']['rgb'] = 200
    window = summarize_window(first, last)
    assert window['received_hz']['depth'] == 30
    assert window['received_hz']['rgb'] == 10
    assert window['published_pairs_hz'] == 5
    camera = validate_cameras([{'id': 'front', 'serial_no': 'TEST_ONLY'}])[0]
    check = CameraCheck(camera)
    check.pipeline(first)
    check.pipeline(last)
    result = check.report(60)
    assert result['pipeline_window'] == window
    assert '成组发布=5.00 Hz' in console_report({'status': 'FAIL', 'data_origin': '合成数据',
                                          'duration_sec': 60, 'cameras': [result]})
    restarted = {**first, 'instance_id': 'second-process'}
    check.pipeline(restarted)
    assert check.report(60)['pipeline_window'] is None
    assert summarize_window(last, restarted) is None
    with pytest.raises(ValueError):
        check.pipeline({**restarted, 'uptime_sec': float('nan')})


def test_adapter_reports_missing_rgb_on_eviction_and_preserves_complete_pair():
    pytest.importorskip('rclpy')
    from sensor_msgs.msg import Image
    from realsense2_camera_msgs.msg import Metadata
    from std_msgs.msg import Header
    path = Path(__file__).resolve().parents[1] / 'scripts/timestamp_adapter.py'
    spec = importlib.util.spec_from_file_location('diagnostic_timestamp_adapter', path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    stats = PipelineDiagnostics('front', 'test-instance')
    images, metadata = [], []
    node = Obj(source='front', limit=64*1024*1024, pending=OrderedDict(), lineage=OrderedDict(), infos={},
               cloud_pending=OrderedDict(), cloud_bytes=0, cloud_limit=16*1024*1024,
               bytes=0, seq=0, epoch=0, last_sensor=None, clock_id='test-instance', config_id='TEST_ONLY',
               diagnostics=stats, get_clock=lambda: Obj(now=lambda: Obj(nanoseconds=1_780_000_000_100_000_000)),
               pair_pub=Obj(publish=images.append), meta_pub=Obj(publish=metadata.append))

    def messages(index):
        h = Header(frame_id='optical')
        h.stamp.sec, h.stamp.nanosec = divmod(1_780_000_000_000_000_000 + index*33_333_333, 10**9)
        raw = {'clock_domain': 'global_time', 'frame_timestamp': str(h.stamp.sec * 1000),
               'sensor_timestamp': 100000 + index*33333, 'hw_timestamp': 102000 + index*33333,
               'frame_number': index, 'actual_exposure': 4000}
        return {'rgb': Image(header=h, width=1, height=1, encoding='rgb8', step=3, data=b'\x01\x02\x03'),
                'depth': Image(header=h, width=1, height=1, encoding='16UC1', step=2, data=b'\x04\x00'),
                'rgb_meta': Metadata(header=h, json_data=json.dumps(raw)),
                'depth_meta': Metadata(header=h, json_data=json.dumps(raw))}

    for index in range(40):
        for name, message in messages(index).items():
            if name != 'rgb':
                adapter.TimestampAdapter.receive(node, name, message)
    snapshot = stats.snapshot(pending_groups=len(node.pending), pending_bytes=node.bytes, clock_epoch=0)
    assert snapshot['received']['depth'] == 40
    assert snapshot['received']['rgb'] == 0
    assert snapshot['published_pairs'] == 0
    assert snapshot['missing_when_discarded'] == {'rgb': 8}
    assert snapshot['discarded_groups'] == {'buffer_limit': 8}
    pair = messages(41)
    for name in ('depth_meta', 'rgb', 'rgb_meta', 'depth'):
        adapter.TimestampAdapter.receive(node, name, pair[name])
    assert stats.published == 1 and len(images) == 1
    assert bytes(images[0].rgb.data) == b'\x01\x02\x03'
    assert bytes(images[0].depth.data) == b'\x04\x00'
    produced = json.loads(metadata[0].json_data)
    raw = json.loads(pair['rgb_meta'].json_data)
    assert produced['rgb']['raw_metadata'] == raw
    assert adapter.ns(images[0].rgb.header) == adapter.midpoint(raw)[0]
    # A late cloud with no image parts does not inflate missing-image counts.
    before = dict(stats.missing)
    stats.discard('buffer_limit', {'cloud': object()})
    assert dict(stats.missing) == before


def test_cloud_backlog_cannot_evict_incomplete_rgbd_groups():
    pytest.importorskip('rclpy')
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    path = Path(__file__).resolve().parents[1] / 'scripts/timestamp_adapter.py'
    spec = importlib.util.spec_from_file_location('cloud_isolation_adapter', path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    stats = PipelineDiagnostics('front', 'test-instance')
    node = Obj(lineage={}, cloud_pending=OrderedDict(), cloud_bytes=0, cloud_limit=16,
               pending=OrderedDict({1: ({'rgb_meta': object()}, 10)}), bytes=10, diagnostics=stats)
    pending = list(node.pending.items())
    for i in range(20):
        h = Header(); h.stamp.nanosec = i
        adapter.TimestampAdapter.receive_cloud(node, PointCloud2(header=h, data=bytes(8)))
    assert list(node.pending.items()) == pending and node.bytes == 10
    assert len(node.cloud_pending) == 2 and node.cloud_bytes == 16
    assert stats.discarded['cloud_buffer_limit'] == 18
    assert not stats.missing
