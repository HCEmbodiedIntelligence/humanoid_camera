"""Reproduce D435 metadata in native UVC units without hiding timing failures."""
import copy
import json

import pytest

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.exposure import metadata_exposure_us
from humanoid_camera.health_check import CameraCheck, expected_parameters, mapped_midpoint_ns
from humanoid_camera.health_report import assemble, console_report, write_report


@pytest.mark.parametrize('model', ['d435', 'D435', 'd435i', 'd435f', 'd435if'])
def test_d435_metadata_exposure_uses_uvc_ticks_only_for_rgb(model):
    raw = {'actual_exposure': 39}
    assert metadata_exposure_us(model, 'rgb', raw) == 3900
    assert raw['actual_exposure'] == 39
    assert metadata_exposure_us(model, 'depth', {'actual_exposure': 3900}) == 3900
    assert metadata_exposure_us('d405', 'rgb', {'actual_exposure': 3900}) == 3900


@pytest.mark.parametrize('invalid', [0, -1, True, None, float('nan'), float('inf')])
def test_invalid_metadata_cannot_be_converted_to_valid_exposure(invalid):
    with pytest.raises((TypeError, ValueError)):
        metadata_exposure_us('d435', 'rgb', {'actual_exposure': invalid})


def observed_pair_check(rgb_ticks=39, skew_us=1949):
    camera = validate_cameras([{'id': 'head', 'device_type': 'd435'}])[0]
    check = CameraCheck(camera, require_equal_exposure=True)
    for i in range(60):
        header = 1_780_000_000_000_000_000 + i * 33_333_333
        raw = {'frame_number': i + 1, 'clock_domain': 'global_time',
               'frame_timestamp': f'{header // 1_000_000}.{header % 1_000_000:06d}',
               'hw_timestamp': 100000 + i * 33333,
               'sensor_timestamp': 98050 + i * 33333,
               'actual_exposure': 3900, 'gain_level': 64, 'auto_exposure': 0}
        color = {**raw, 'sensor_timestamp': raw['sensor_timestamp'] + skew_us, 'actual_exposure': rgb_ticks}
        before = copy.deepcopy(color)
        for stream, metadata in [('rgb', color), ('depth', raw)]:
            check.raw(stream, metadata, header, header + 5_000_000, i / 30)
        rgb_time, depth_time = mapped_midpoint_ns(color), mapped_midpoint_ns(raw)
        check.normalized({'source_id': 'head', 'clock_epoch': 0, 'capture_time_ns': rgb_time,
                          'driver_header_timestamp_ns': header,
                          'rgb': {'source_seq': i + 1, 'capture_time_ns': rgb_time},
                          'depth': {'source_seq': i + 1, 'capture_time_ns': depth_time}}, rgb_time, i / 30)
        assert color == before
    return check


def test_39_vs_3900_is_equal_exposure_but_1949us_skew_still_fails(tmp_path):
    check = observed_pair_check()
    report = assemble([check], 2., {'head': {**expected_parameters(check.camera), 'use_sim_time': False}},
                      verify_images=False, data_origin='合成数据自测')
    result = report['cameras'][0]
    assert result['metrics']['rgb_exposure_us']['max'] == 3900
    assert result['metrics']['rgb_depth_exposure_difference_us']['max'] == 0
    assert result['metrics']['rgb_depth_midpoint_skew_ms']['max'] == 1.949
    assert result['failures'] == {'RGB与深度曝光中点差超过配置阈值': 60}
    assert result['status'] == 'FAIL'
    assert result['raw_metadata_samples']['rgb']['last']['raw']['actual_exposure'] == 39
    assert result['metrics']['rgb_sensor_minus_hw_us']['max'] == -1
    assert result['metrics']['depth_sensor_minus_hw_us']['max'] == -1950
    assert '原始值×100' in console_report(report)
    path = write_report(report, tmp_path)
    saved = json.loads((path / 'report.json').read_text())['cameras'][0]
    assert saved['metadata_exposure_scale_us'] == {'rgb': 100, 'depth': 1}


def test_conversion_does_not_hide_exposure_above_limit_or_pair_mismatch():
    check = observed_pair_check(rgb_ticks=60, skew_us=100)
    result = check.report(2., {**expected_parameters(check.camera), 'use_sim_time': False})
    assert result['metrics']['rgb_exposure_us']['max'] == 6000
    assert result['failures']['rgb:实际曝光超过要求上限'] == 60
    assert result['failures']['RGB与深度实际曝光时长不一致'] == 60
