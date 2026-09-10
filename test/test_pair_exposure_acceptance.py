"""Compare exposure duration for the same RGB/depth pair, independently of timing."""
import csv
import json

import pytest

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters
from humanoid_camera.health_report import assemble, console_report, write_report


def observe(exposures, *, require_equal=True, skew_us=100):
    camera = validate_cameras([{'id': 'front', 'device_type': 'd405'}])[0]
    check = CameraCheck(camera, verify_images=True, require_equal_exposure=require_equal)
    for index in range(60):
        header = 1_780_000_000_000_000_000 + index * 33_333_333
        midpoint = header - 2_000_000
        rgb_exposure, depth_exposure = exposures[index % len(exposures)]
        raw = {'clock_domain': 'global_time',
               'frame_timestamp': f'{header // 1_000_000}.{header % 1_000_000:06d}',
               'hw_timestamp': 100000 + index * 33333,
               'sensor_timestamp': 98000 + index * 33333,
               'frame_number': index + 1, 'gain_level': 16, 'auto_exposure': 1}
        for stream, exposure, offset in [('rgb', rgb_exposure, 0), ('depth', depth_exposure, skew_us)]:
            metadata = {**raw, 'sensor_timestamp': raw['sensor_timestamp'] + offset}
            if exposure is not None:
                metadata['actual_exposure'] = exposure
            check.raw(stream, metadata, header, header + 5_000_000, index / 30)
        # Raw pairs must be evaluated once even if normalized data arrives later.
        normalized = {'source_id': 'front', 'clock_epoch': 0,
                      'capture_time_ns': midpoint, 'driver_header_timestamp_ns': header,
                      'rgb': {'capture_time_ns': midpoint, 'source_seq': index + 1},
                      'depth': {'capture_time_ns': midpoint + skew_us * 1000, 'source_seq': index + 1}}
        check.normalized(normalized, midpoint, index / 30)
        check.image(midpoint, midpoint, midpoint + skew_us * 1000, index / 30)
    parameters = {**expected_parameters(camera), 'use_sim_time': False}
    return assemble([check], 2., {'front': parameters}, verify_images=True,
                    data_origin='合成数据自测')


def test_pairwise_equal_dynamic_exposures_pass_with_real_midpoint_difference():
    report = observe([(4000, 4000), (3000, 3000)])
    camera = report['cameras'][0]
    assert report['status'] == 'PASS'
    assert camera['metrics']['rgb_depth_exposure_difference_us']['max'] == 0
    assert camera['metrics']['rgb_depth_midpoint_skew_ms']['max'] == .1
    assert camera['counts']['exposure_pairs'] == camera['counts']['raw_pairs'] == 60
    assert camera['limits']['rgb_depth_exposure_difference_us'] == 0
    assert report['physical_clock_accuracy_verified'] is False


def test_equal_average_and_maximum_cannot_hide_pairwise_exposure_mismatch():
    report = observe([(4000, 3000), (3000, 4000)])
    camera = report['cameras'][0]
    assert camera['metrics']['rgb_exposure_us'] == camera['metrics']['depth_exposure_us']
    assert report['status'] == 'FAIL'
    assert camera['failures']['RGB与深度实际曝光时长不一致'] == 60
    assert camera['metrics']['rgb_depth_exposure_difference_us']['max'] == 1000


def test_equality_check_never_silently_allows_rounding_tolerance():
    report = observe([(4000, 4001)])
    assert report['status'] == 'FAIL'
    assert report['cameras'][0]['metrics']['rgb_depth_exposure_difference_us']['max'] == 1


def test_duration_equality_does_not_make_unsynchronized_exposures_pass():
    report = observe([(4000, 4000)], skew_us=2000)
    camera = report['cameras'][0]
    assert report['status'] == 'FAIL'
    assert camera['metrics']['rgb_depth_exposure_difference_us']['max'] == 0
    assert camera['failures']['RGB与深度曝光中点差超过配置阈值'] == 60


@pytest.mark.parametrize('invalid', [None, 0, -1, True, float('nan'), float('inf')])
def test_invalid_exposure_never_passes_or_breaks_report_serialization(invalid, tmp_path):
    report = observe([(4000, invalid)])
    camera = report['cameras'][0]
    assert report['status'] == 'UNKNOWN'
    assert 'rgb_depth_exposure_difference_us' not in camera['metrics']
    assert camera['unknowns']['RGB-D配对缺少有效实际曝光，无法确认时长一致'] == 60
    path = write_report(report, tmp_path)
    assert json.loads((path / 'report.json').read_text())['status'] == 'UNKNOWN'
    with (path / 'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert all(row['exposure_difference_us'] == '' for row in rows)


def test_optional_measurement_reports_differences_without_claiming_equality(tmp_path):
    report = observe([(4000, 3000)], require_equal=False)
    camera = report['cameras'][0]
    assert report['status'] == 'PASS'
    assert camera['limits']['rgb_depth_exposure_difference_us'] is None
    assert '仅报告差值，未要求相等' in console_report(report)
    assert camera['metrics']['rgb_depth_exposure_difference_us']['max'] == 1000
    path = write_report(report, tmp_path)
    assert '仅统计差值，未要求相等' in (path / 'report.html').read_text()


def test_strict_report_preserves_duration_difference_and_midpoint_difference(tmp_path):
    report = observe([(4000, 3000)])
    path = write_report(report, tmp_path)
    assert 'RGB-D曝光时长差 max us' in console_report(report)
    page = (path / 'report.html').read_text()
    assert '要求每对实际曝光时长差为 0 μs' in page
    assert 'RGB/深度实际曝光时长差（μs）' in page
    with (path / 'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 60
    assert all(float(row['exposure_difference_us']) == 1000 for row in rows)
    assert all(float(row['skew_ms']) == .1 for row in rows)
