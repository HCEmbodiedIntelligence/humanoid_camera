"""Sample real RGBD messages into lossless PNGs and frame-specific evidence."""
from collections import OrderedDict
import hashlib
import json
import math
from pathlib import Path
import queue
import threading

import numpy as np
from PIL import Image

from .exposure import metadata_exposure_us
from .depth_visualization import color_scale, depth_color_preview, depth_color_legend


def stamp(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def safe_json(value):
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def pixels(message, *, depth=False):
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step <= 0 or step * height > 64 * 1024 * 1024:
        raise ValueError('证据图像尺寸无效或超过64 MiB')
    if len(message.data) < step * height:
        raise ValueError('证据图像数据不足')
    rows = np.frombuffer(message.data, dtype=np.uint8, count=step * height).reshape(height, step)
    if depth:
        if message.encoding not in ('16UC1', 'mono16') or step < width * 2:
            raise ValueError('深度证据需要16位单通道图像')
        data = rows[:, :width * 2].copy()
        return data.view('>u2' if message.is_bigendian else '<u2').reshape(height, width).astype(np.uint16)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(message.encoding)
    if channels is None or step < width * channels:
        raise ValueError('不支持的RGB证据编码: ' + message.encoding)
    data = rows[:, :width * channels].reshape(height, width, channels)
    if channels == 1:
        return data[:, :, 0].copy()
    if message.encoding.startswith('bgr'):
        data = data[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    return data.copy()


def depth_preview(depth):
    valid = depth[depth != 0]
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if valid.size == 0:
        return preview, {'min_raw': None, 'max_raw': None, 'zero_is_invalid': True}
    low, high = (float(value) for value in np.percentile(valid, (2, 98)))
    if high <= low:
        preview[depth != 0] = 128
    else:
        scaled = 32 + np.clip((depth.astype(np.float32) - low) / (high - low), 0, 1) * 223
        preview[depth != 0] = scaled[depth != 0].astype(np.uint8)
    return preview, {'min_raw': low, 'max_raw': high, 'zero_is_invalid': True}


class EvidenceRecorder:
    def __init__(self, directory, *, interval=10., max_pairs=6, depth_min_m=.2, depth_max_m=2.):
        self.directory = Path(directory)
        self.depth_scale = color_scale(depth_min_m, depth_max_m)
        self.interval, self.max_pairs = interval, max_pairs
        self.last_selected, self.selected = {}, {}
        self.metadata, self.pending = OrderedDict(), OrderedDict()
        self.saved, self.errors = [], []
        self.jobs = queue.Queue(maxsize=2)
        self.worker = threading.Thread(target=self._worker, name='camera-evidence-writer', daemon=True)
        self.worker.start()

    def metadata_received(self, camera, data, header_ns):
        key = (camera['id'], header_ns)
        self.metadata[key] = data
        while len(self.metadata) > 256:
            self.metadata.popitem(last=False)
        if key in self.pending:
            self._submit(key)

    def image_received(self, camera, message, elapsed):
        ident = camera['id']
        if self.selected.get(ident, 0) >= self.max_pairs or elapsed - self.last_selected.get(ident, -math.inf) < self.interval:
            return
        self.flush(elapsed)
        size = len(message.rgb.data) + len(message.depth.data)
        if size > 64 * 1024 * 1024:
            self.errors.append(ident + ': RGBD证据超过64 MiB，未保存')
            self.last_selected[ident] = elapsed
            return
        # Bound retained image data even if matching metadata never arrives.
        while self.pending and sum(item['size'] for item in self.pending.values()) + size > 64 * 1024 * 1024:
            self._submit(next(iter(self.pending)))
        index = self.selected.get(ident, 0) + 1
        self.selected[ident], self.last_selected[ident] = index, elapsed
        key = (ident, stamp(message.header))
        self.pending[key] = {'camera': camera, 'message': message, 'elapsed': elapsed,
                             'index': index, 'size': size}
        if key in self.metadata:
            self._submit(key)

    def flush(self, elapsed=math.inf):
        for key, item in list(self.pending.items()):
            if elapsed - item['elapsed'] >= 2.:
                self._submit(key)

    def _submit(self, key):
        item = self.pending.pop(key)
        item['metadata'] = self.metadata.pop(key, None)
        try:
            self.jobs.put_nowait(item)
        except queue.Full:
            self.errors.append(key[0] + ': 证据写入队列已满，跳过抽样照片')

    def _worker(self):
        while True:
            item = self.jobs.get()
            try:
                if item is None:
                    return
                self.saved.append(self._save(item))
            except Exception as error:
                self.errors.append(item['camera']['id'] + ': 证据保存失败: ' + str(error))
            finally:
                self.jobs.task_done()

    def _save(self, item):
        camera, message, metadata = item['camera'], item['message'], item['metadata']
        ident = camera['id']
        relative = Path('evidence') / ident / f"{item['index']:03d}"
        directory = self.directory / relative
        directory.mkdir(parents=True, exist_ok=False)
        errors = []
        files = {}
        preview_range = None
        for stream in ('rgb', 'depth'):
            try:
                data = pixels(getattr(message, stream), depth=stream == 'depth')
                Image.fromarray(data).save(directory / (stream + '.png'), compress_level=1)
                files[stream] = str(relative / (stream + '.png'))
                if stream == 'depth':
                    preview, preview_range = depth_preview(data)
                    Image.fromarray(preview).save(directory / 'depth_preview.png', compress_level=1)
                    files['depth_preview'] = str(relative / 'depth_preview.png')
                    colored, _ = depth_color_preview(data, self.depth_scale['min_m'], self.depth_scale['max_m'])
                    Image.fromarray(colored).save(directory / 'depth_color.png', compress_level=1)
                    files['depth_color'] = str(relative / 'depth_color.png')
                    depth_color_legend(self.depth_scale['min_m'], self.depth_scale['max_m'], data.shape[1]).save(
                        directory / 'depth_color_legend.png', compress_level=1)
                    files['depth_color_legend'] = str(relative / 'depth_color_legend.png')
            except Exception as error:
                errors.append(stream + ': ' + str(error))
        matched = (isinstance(metadata, dict) and metadata.get('source_id') == ident
                   and metadata.get('capture_time_ns') == stamp(message.header)
                   and all(isinstance(metadata.get(stream), dict)
                           and metadata[stream].get('capture_time_ns') == stamp(getattr(message, stream).header)
                           for stream in ('rgb', 'depth')))
        observations = {}
        for stream in ('rgb', 'depth'):
            image = getattr(message, stream)
            raw = metadata[stream].get('raw_metadata', {}) if matched else {}
            if not isinstance(raw, dict):
                raw = {}
            try:
                exposure = metadata_exposure_us(camera['device_type'], stream, raw)
            except (TypeError, ValueError, KeyError, OverflowError):
                exposure = None
            observations[stream] = {'header_ns': stamp(image.header), 'frame_id': image.header.frame_id,
                                    'width': image.width, 'height': image.height, 'encoding': image.encoding,
                                    'source_data_sha256': hashlib.sha256(image.data).hexdigest(),
                                    'frame_number': raw.get('frame_number'), 'raw_actual_exposure': raw.get('actual_exposure'),
                                    'actual_exposure_us': exposure, 'gain': raw.get('gain_level')}
        result = {'camera_id': ident, 'device_type': camera['device_type'], 'serial_no': camera['serial_no'],
                  'sample_index': item['index'], 'elapsed_sec': item['elapsed'], 'topic': camera['rgbd_topic'],
                  'pair_header_ns': stamp(message.header), 'observations': observations,
                  'metadata_status': 'matched' if matched else 'missing_or_mismatched',
                  'depth_preview_range': preview_range, 'files': files, 'errors': errors,
                  'depth_color_scale': self.depth_scale,
                  'pixel_processing': 'RGB PNG lossless; depth PNG preserves received uint16 values; preview is scaled for display',
                  'camera_config_id': metadata.get('camera_config_id') if matched else None,
                  'configuration': {key: camera[key] for key in (
                      'width', 'height', 'fps', 'depth_auto_exposure', 'color_auto_exposure',
                      'depth_exposure_us', 'color_exposure_us', 'depth_auto_gain', 'color_auto_gain',
                      'depth_auto_gain_limit', 'color_auto_gain_limit', 'rgbd_max_midpoint_skew_ms')
                      if key in camera}}
        result['sha256'] = {name: hashlib.sha256((self.directory / path).read_bytes()).hexdigest()
                            for name, path in files.items()}
        evidence_file = relative / 'frame.json'
        (self.directory / evidence_file).write_text(json.dumps(safe_json({**result, 'source_metadata': metadata}),
                                                              ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        result['files']['metadata'] = str(evidence_file)
        self.errors.extend(ident + ': ' + error for error in errors)
        return safe_json(result)

    def close(self):
        self.flush()
        self.jobs.put(None)
        self.worker.join()
        return {'enabled': True, 'interval_sec': self.interval, 'max_pairs_per_camera': self.max_pairs,
                'selected': self.selected, 'saved': sorted(self.saved, key=lambda item: (item['camera_id'], item['sample_index'])),
                'errors': self.errors}
