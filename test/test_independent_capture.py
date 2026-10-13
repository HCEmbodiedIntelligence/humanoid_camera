"""Disabling SDK synchronization must really bypass it and preserve source frames."""
from collections import OrderedDict
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as Obj

import pytest
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from realsense2_camera_msgs.msg import ImageMetadata, RGBDMetadata
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.depth_processing import INDEPENDENT_FILTER_PARAMETERS
from humanoid_camera.exposure_pairs import ExposurePairs
from humanoid_camera.health_check import expected_parameters
from humanoid_camera.pipeline_diagnostics import PipelineDiagnostics


def module(relative):
    path = Path(__file__).resolve().parents[1] / relative
    spec = importlib.util.spec_from_file_location('independent_capture_test_' + path.stem, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('model,depth', [('d405', True), ('d435', True), ('d435', False)])
def test_sync_off_survives_save_and_disables_implicit_filter_sync(model, depth):
    overrides = {**{key: True for key in INDEPENDENT_FILTER_PARAMETERS},
                 'enable_sync': True, 'spatial_filter': {'enable': True, 'smooth_alpha': .6}}
    camera = validate_cameras([{'id': 'front', 'device_type': model,
        'sync_rgb_depth': False, 'depth_enabled': depth, 'parameters': overrides}])[0]
    assert camera['sync_rgb_depth'] is False
    assert validate_cameras([camera])[0] == camera
    launch = module('launch/realsense_camera.launch.py')
    # Also test overrides injected after validation, as in old saved profiles.
    actual = evaluate_parameters(LaunchContext(), normalize_parameters([
        launch._parameters({**camera, 'parameters': overrides})]))[0]
    assert actual['enable_sync'] is False
    assert actual['enable_depth'] is depth
    assert actual['enable_color'] is True
    assert actual['spatial_filter.smooth_alpha'] == .6
    for key in INDEPENDENT_FILTER_PARAMETERS:
        assert actual[key] is expected_parameters(camera)[key] is False


def packet(frame, header_ns, sensor_us):
    header = Header(frame_id='optical')
    header.stamp.sec, header.stamp.nanosec = divmod(header_ns, 10**9)
    return ImageMetadata(image=Image(header=header, width=1, height=1, encoding='rgb8',
        step=3, data=b'\x01\x02\x03'), camera_info=CameraInfo(header=deepcopy(header)),
        json_data=json.dumps({'clock_domain': 'global_time', 'frame_timestamp': '1780000000000',
            'hw_timestamp': 100000, 'sensor_timestamp': sensor_us,
            'frame_number': frame, 'actual_exposure': 4000}))


def test_independent_headers_associate_only_within_exposure_limit_and_preserve_packets():
    adapter = module('scripts/timestamp_adapter.py')
    stats = PipelineDiagnostics('front', 'test')
    outputs = []
    node = Obj(source='front', device_type='d405', depth_enabled=True, sdk_sync=False,
        bundled_input=True, independent_pairs=ExposurePairs(2_000_000, 4096, stats),
        lineage=OrderedDict(), infos={}, cloud_pending=OrderedDict(), cloud_bytes=0,
        seq=0, epoch=0, last_sensor=None, clock_id='test', config_id='test', diagnostics=stats,
        get_clock=lambda: Obj(now=lambda: Obj(nanoseconds=1780000000100000000)),
        packet_type=RGBDMetadata, packet_pub=Obj(publish=outputs.append),
        pair_pub=Obj(get_subscription_count=lambda: 0), meta_pub=Obj(get_subscription_count=lambda: 0))
    rgb = packet(100, 1780000000000000000, 100000)
    too_late = packet(101, 1780000000033333333, 133333)
    depth = packet(100, 1780000000000500000, 100100)
    originals = [deepcopy(value) for value in (rgb, too_late, depth)]
    adapter.TimestampAdapter.receive_packet(node, 'rgb', rgb)
    adapter.TimestampAdapter.receive_packet(node, 'depth', too_late)
    assert not outputs
    adapter.TimestampAdapter.receive_packet(node, 'depth', depth)
    assert len(outputs) == 1
    evidence = json.loads(outputs[0].json_data)
    assert evidence['pairing_method'] == 'independent_exposure_midpoint_nearest'
    assert evidence['sdk_sync'] is False
    for name, source in [('rgb', rgb), ('depth', depth)]:
        assert evidence[name]['original_header_ns'] == adapter.ns(source.image.header)
        assert evidence[name]['source_seq'] == 100
        assert evidence[name]['raw_metadata'] == json.loads(source.json_data)
        assert adapter.ns(getattr(outputs[0].rgbd, name).header) == adapter.midpoint(json.loads(source.json_data))[0]
    assert [rgb, too_late, depth] == originals
    assert len(node.independent_pairs.pending) == 1


def test_missing_counterpart_is_bounded_without_fabricating_frames():
    stats = PipelineDiagnostics('front', 'test')
    pairs = ExposurePairs(2_000_000, 4096, stats)
    for index in range(100):
        assert pairs.add('rgb', packet(index, 1780000000000000000, index * 33333), index * 33333333) is None
        assert pairs.bytes <= 4096 and len(pairs.pending) <= 32
    assert stats.discarded['independent_pair_buffer_limit'] > 0
    pairs.clear('clock_reset')
    assert not pairs.pending and pairs.bytes == 0
