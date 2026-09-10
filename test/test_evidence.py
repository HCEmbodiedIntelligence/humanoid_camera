"""Lossless evidence export, exact frame metadata matching and failure retention."""
import copy
import hashlib
import json
from types import SimpleNamespace as Obj
import zipfile

import numpy as np
from PIL import Image
import pytest

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.evidence import EvidenceRecorder, pixels
from humanoid_camera.health_report import bundle_report, write_report


def frame(offset=0, bigendian=False):
    camera = validate_cameras([{'id': 'camera_hand', 'device_type': 'd435', 'serial_no': 'TEST_ONLY'}])[0]
    t = 1_780_000_000_000_000_000 + offset
    def header(value):
        return Obj(stamp=Obj(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000), frame_id='optical')
    colors = bytes([255, 0, 0, 0, 255, 0, 99, 99, 0, 0, 255, 127, 128, 129, 99, 99])
    depth = np.array([[0, 3900], [65535, 258]], dtype='>u2' if bigendian else '<u2')
    padded = b''.join(row.tobytes() + b'\x99\x99' for row in depth)
    image = Obj(header=header(t),
                rgb=Obj(header=header(t), width=2, height=2, step=8, encoding='rgb8', data=colors, is_bigendian=0),
                depth=Obj(header=header(t - 1949000), width=2, height=2, step=6, encoding='16UC1', data=padded, is_bigendian=int(bigendian)))
    metadata = {'source_id': camera['id'], 'capture_time_ns': t, 'pair_seq': 12,
                'rgb': {'capture_time_ns': t, 'raw_metadata': {'frame_number': 20, 'actual_exposure': 39, 'gain_level': 60}},
                'depth': {'capture_time_ns': t - 1949000, 'raw_metadata': {'frame_number': 17, 'actual_exposure': 3900, 'gain_level': 64}}}
    return camera, image, metadata, t


@pytest.mark.parametrize('metadata_first', [True, False])
@pytest.mark.parametrize('bigendian', [True, False])
def test_saved_photos_preserve_pixels_and_record_matching_frame_evidence(tmp_path, metadata_first, bigendian):
    camera, image, metadata, t = frame(bigendian=bigendian)
    original = copy.deepcopy((image, metadata))
    writer = EvidenceRecorder(tmp_path)
    if metadata_first:
        writer.metadata_received(camera, metadata, t)
    writer.image_received(camera, image, 2.)
    if not metadata_first:
        writer.metadata_received(camera, metadata, t)
    result = writer.close()
    assert not result['errors']
    assert len(result['saved']) == 1
    saved = result['saved'][0]
    assert saved['metadata_status'] == 'matched'
    assert saved['observations']['rgb']['actual_exposure_us'] == 3900
    assert saved['observations']['rgb']['raw_actual_exposure'] == 39
    assert saved['observations']['rgb']['frame_number'] == 20
    assert saved['observations']['depth']['frame_number'] == 17
    assert saved['observations']['rgb']['header_ns'] - saved['observations']['depth']['header_ns'] == 1949000
    rgb = np.array(Image.open(tmp_path / saved['files']['rgb']))
    np.testing.assert_array_equal(rgb, [[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [127, 128, 129]]])
    depth = np.array(Image.open(tmp_path / saved['files']['depth']))
    np.testing.assert_array_equal(depth, [[0, 3900], [65535, 258]])
    assert saved['observations']['depth']['source_data_sha256'] == hashlib.sha256(image.depth.data).hexdigest()
    for name, digest in saved['sha256'].items():
        assert hashlib.sha256((tmp_path / saved['files'][name]).read_bytes()).hexdigest() == digest
    manifest = json.loads((tmp_path / saved['files']['metadata']).read_text())
    assert manifest['source_metadata'] == metadata
    assert (image, metadata) == original


@pytest.mark.parametrize('mismatch', ['missing', 'wrong_source', 'wrong_frame'])
def test_missing_or_mismatched_metadata_keeps_photo_but_exposure_unknown(tmp_path, mismatch):
    camera, image, metadata, t = frame()
    writer = EvidenceRecorder(tmp_path)
    writer.image_received(camera, image, 0.)
    if mismatch != 'missing':
        if mismatch == 'wrong_source':
            metadata['source_id'] = 'another_camera'
        else:
            metadata['depth']['capture_time_ns'] += 1
        writer.metadata_received(camera, metadata, t)
    result = writer.close()
    saved = result['saved'][0]
    assert (tmp_path / saved['files']['rgb']).is_file()
    assert saved['metadata_status'] == 'missing_or_mismatched'
    assert saved['observations']['rgb']['actual_exposure_us'] is None


def test_sampling_interval_and_pair_limit_bound_image_capture(tmp_path):
    writer = EvidenceRecorder(tmp_path, interval=10., max_pairs=2)
    for second in (0, 1, 10, 11, 20):
        camera, image, metadata, t = frame(offset=second * 1_000_000_000)
        writer.metadata_received(camera, metadata, t)
        writer.image_received(camera, image, second)
    result = writer.close()
    assert [sample['elapsed_sec'] for sample in result['saved']] == [0, 10]


def test_failed_depth_export_keeps_rgb_and_is_visible_in_report_and_zip(tmp_path):
    camera, image, metadata, t = frame()
    image.depth.encoding = '32FC1'
    writer = EvidenceRecorder(tmp_path)
    writer.metadata_received(camera, metadata, t)
    writer.image_received(camera, image, 0.)
    evidence = writer.close()
    assert evidence['errors']
    assert 'rgb' in evidence['saved'][0]['files']
    assert 'depth' not in evidence['saved'][0]['files']
    report = {'status': 'FAIL', 'data_origin': '合成数据自测', 'duration_sec': 1.,
              'verify_images': True, 'cameras': [], 'evidence': evidence}
    write_report(report, tmp_path, destination=tmp_path)
    html = (tmp_path / 'report.html').read_text()
    assert '合成数据示例' in html and '真实图像抽样证据' in html
    assert evidence['saved'][0]['files']['rgb'] in html
    archive = bundle_report(tmp_path)
    with zipfile.ZipFile(archive) as zipped:
        assert tmp_path.name + '/report.html' in zipped.namelist()
        assert tmp_path.name + '/' + evidence['saved'][0]['files']['rgb'] in zipped.namelist()


def test_rgb_bgr_and_invalid_size_are_handled_without_modifying_input():
    _, image, _, _ = frame()
    image.rgb.encoding = 'bgr8'
    np.testing.assert_array_equal(pixels(image.rgb)[0, 0], [0, 0, 255])
    image.rgb.step = 2
    with pytest.raises(ValueError):
        pixels(image.rgb)


@pytest.mark.parametrize('ending', ['failed_check', 'interrupt', 'runtime_error'])
def test_command_keeps_received_photos_and_report_on_failure_or_interruption(tmp_path, monkeypatch, capsys, ending):
    import importlib.util
    from pathlib import Path
    import sys
    rclpy = pytest.importorskip('rclpy')
    path = Path(__file__).resolve().parents[1] / 'scripts/check_cameras.py'
    spec = importlib.util.spec_from_file_location('camera_evidence_command', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    camera, image, metadata, timestamp = frame()
    metadata.update(clock_epoch=0, driver_header_timestamp_ns=timestamp)
    subscriptions, state = {}, {'elapsed': 0., 'calls': 0, 'destroyed': False}

    class Node:
        def create_client(self, kind, topic):
            return Obj(service_is_ready=lambda: False)

        def create_subscription(self, kind, topic, callback, qos):
            subscriptions[topic] = callback

        def destroy_node(self):
            state['destroyed'] = True

    def spin_once(node, timeout_sec):
        state['calls'] += 1
        state['elapsed'] = state['calls'] * .1
        if state['calls'] == 1:
            subscriptions[camera['rgbd_topic']](image)
            subscriptions[camera['metadata_topic']](Obj(header=image.header, json_data=json.dumps(metadata)))
        elif ending == 'interrupt':
            raise KeyboardInterrupt
        elif ending == 'runtime_error':
            raise RuntimeError('TEST_ONLY receiver failure')
        else:
            state['elapsed'] = 1.1

    config = tmp_path / 'cameras.yaml'
    config.write_text(json.dumps({'schema_version': 1, 'cameras': [camera]}))
    monkeypatch.setattr(module.time, 'monotonic', lambda: state['elapsed'])
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'create_node', lambda name: Node())
    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(sys, 'argv', [str(path), '--config', str(config), '--duration', '1', '--warmup', '0',
                                    '--save-evidence', '--output', str(tmp_path / 'checks')])
    assert module.main() == (130 if ending == 'interrupt' else 1)
    assert state['destroyed']
    report_file, = (tmp_path / 'checks').glob('*/report.json')
    report = json.loads(report_file.read_text())
    assert report['status'] != 'PASS' and report['verify_images'] is True
    assert report['interrupted'] == (ending == 'interrupt')
    if ending == 'runtime_error':
        assert report['runtime_error'] == 'TEST_ONLY receiver failure'
    saved, = report['evidence']['saved']
    assert saved['metadata_status'] == 'matched'
    assert (report_file.parent / saved['files']['rgb']).is_file()
    assert (report_file.parent / saved['files']['depth']).is_file()
    archive, = (tmp_path / 'checks').glob('*.zip')
    with zipfile.ZipFile(archive) as zipped:
        assert report_file.parent.name + '/' + saved['files']['rgb'] in zipped.namelist()
    assert '汇报压缩包：' in capsys.readouterr().out
