"""Distinguish a diagnostic subscriber's metadata gap from an absent RGBD pair."""
import copy
import csv
import json

from humanoid_camera.frame_trace import FrameTrace
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters
from humanoid_camera.health_report import assemble, console_report, write_report


def normalized(counter, source='front', epoch=0):
    return {'source_id': source, 'clock_id': 'test_clock', 'clock_epoch': epoch,
            'receive_time_ns': counter * 100 + 20,
            'rgb': {'capture_time_ns': counter * 100, 'source_seq': counter,
                    'raw_metadata': {'frame_number': counter, 'time_of_arrival': 1}},
            'depth': {'capture_time_ns': counter * 100 - 1, 'source_seq': counter,
                      'raw_metadata': {'frame_number': counter, 'time_of_arrival': 1}}}


def raw(trace, counter):
    for stream in ('rgb', 'depth'):
        trace.raw(stream, {'frame_number': counter}, counter * 100, counter * 100 + 10, counter / 30)


def pair(trace, counter, *, source='front', epoch=0, image=True):
    trace.normalized(normalized(counter, source, epoch), counter * 100, counter * 100 + 30, counter / 30)
    if image:
        trace.image(counter * 100, counter * 100, counter * 100 - 1, counter * 100 + 40, counter / 30)


def test_recovery_requires_matching_image_headers_and_deduplicates_callbacks():
    trace = FrameTrace('front')
    raw(trace, 10)
    pair(trace, 11)
    pair(trace, 11)
    pair(trace, 12, image=False)
    trace.image(1200, 1200, 1198, 1240, .4)  # Wrong inner depth Header.
    raw(trace, 13)
    for evidence in trace.report()['gap_evidence'].values():
        assert evidence['raw_gap_count'] == 2
        assert evidence['recovered_frame_numbers'] == [11]
        assert evidence['present_in_normalized_images'] == 1
        assert evidence['unresolved_gap_count'] == 1


def test_warmup_boundaries_wrong_sources_and_inconsistent_identity_are_not_recovery():
    trace = FrameTrace('front')
    raw(trace, 10)
    pair(trace, 9)
    pair(trace, 14)
    pair(trace, 11, source='different_camera')
    wrong = normalized(12)
    wrong['depth']['source_seq'] = 120
    trace.normalized(wrong, 1200, 1230, .4)
    trace.image(1200, 1200, 1199, 1240, .4)
    raw(trace, 13)
    assert all(e['present_in_normalized_images'] == 0 for e in trace.report()['gap_evidence'].values())


def test_truncation_or_clock_change_never_produces_a_loss_classification():
    truncated = FrameTrace('front', limit=3)
    raw(truncated, 10)
    raw(truncated, 12)
    report = truncated.report()
    assert report['classification_status'] == 'trace_truncated'
    assert len(report['events']) == 3 and report['discarded_events'] == 1
    assert report['gap_evidence'] == {}
    changed = FrameTrace('front')
    raw(changed, 10)
    pair(changed, 11, epoch=0)
    pair(changed, 12, epoch=1)
    raw(changed, 13)
    assert changed.report()['classification_status'] == 'clock_changed'
    assert changed.report()['gap_evidence'] == {}


def test_out_of_order_or_invalid_frame_numbers_do_not_create_fake_missing_photos():
    trace = FrameTrace('front')
    raw(trace, 10)
    raw(trace, 12)
    raw(trace, 11)
    assert trace.report()['gap_evidence']['depth']['status'] == 'nonmonotonic_frame_numbers'
    trace.raw('rgb', {'frame_number': True, 'time_of_arrival': float('nan')}, 0, 0, 0.)
    trace.raw('rgb', {'frame_number': 13, 'time_of_arrival': 10 ** 1000}, 0, 0, 0.)
    result = trace.report()
    assert result['gap_evidence']['rgb']['status'] == 'insufficient_frame_numbers'
    json.dumps(result, allow_nan=False)


def test_end_to_end_trace_export_keeps_original_failures_and_original_metadata(tmp_path):
    camera = validate_cameras([{'id': 'front', 'device_type': 'd405'}])[0]
    check = CameraCheck(camera, verify_images=True, require_equal_exposure=True, save_frame_trace=True)
    for index in range(30):
        header = 1_780_000_000_000_000_000 + index * 33_333_333
        midpoint = header - 2_000_000
        raw_data = {'clock_domain': 'global_time', 'frame_timestamp': str(header / 1_000_000),
                    'hw_timestamp': 100000 + index * 33333, 'sensor_timestamp': 98000 + index * 33333,
                    'frame_number': index + 1, 'actual_exposure': 4000, 'gain_level': 16, 'auto_exposure': 1,
                    'time_of_arrival': header // 1_000_000 + 1}
        if index != 12:
            for stream in ('rgb', 'depth'):
                check.raw(stream, raw_data, header, header + 5_000_000, index / 30)
        data = {'source_id': 'front', 'clock_id': 'test_clock', 'clock_epoch': 0,
                'capture_time_ns': midpoint, 'driver_header_timestamp_ns': header,
                'receive_time_ns': header + 2_000_000,
                **{stream: {'capture_time_ns': midpoint, 'source_seq': index + 1, 'raw_metadata': raw_data}
                   for stream in ('rgb', 'depth')}}
        original = copy.deepcopy(data)
        check.normalized(data, midpoint, index / 30, header + 6_000_000)
        check.image(midpoint, midpoint, midpoint, index / 30, header + 7_000_000)
        assert data == original
    params = {**expected_parameters(camera), 'use_sim_time': False}
    report = assemble([check], 1., {'front': params}, verify_images=True, data_origin='合成数据自测')
    assert report['status'] == 'FAIL'
    assert report['cameras'][0]['failures']['rgb:观测帧号缺口'] == 1
    evidence = report['cameras'][0]['frame_trace']['gap_evidence']
    assert evidence['rgb']['recovered_frame_numbers'] == [13]
    assert evidence['depth']['recovered_frame_numbers'] == [13]
    path = write_report(report, tmp_path)
    assert '仍对应已收到的标准化 RGBD 图像' in console_report(report)
    with (path / 'frame_events.csv').open() as f:
        events = list(csv.DictReader(f))
    assert len(events) == 118
    assert {e['kind'] for e in events} == {'raw_rgb', 'raw_depth', 'normalized', 'image'}
    assert all(e['receive_ns'] for e in events)
    assert 'frame_events.csv' in (path / 'report.html').read_text()
    assert json.loads((path / 'report.json').read_text())['status'] == 'FAIL'


def test_trace_is_optional_for_existing_callers():
    camera = validate_cameras([{'id': 'front'}])[0]
    check = CameraCheck(camera)
    assert check.report(1.)['frame_trace'] is None
