"""Read-only measurement checks; reported clock agreement is not physical calibration."""
from collections import Counter, OrderedDict, deque
from decimal import Decimal
import math
from .exposure import rgb_exposure_parameter


def finite(value):
    if isinstance(value, bool):
        raise ValueError('numeric metadata must not be boolean')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('non-finite metadata')
    return number


def mapped_midpoint_ns(raw):
    if raw.get('clock_domain') != 'global_time':
        raise ValueError('clock_domain is not global_time')
    sensor, hardware = int(raw['sensor_timestamp']), int(raw['hw_timestamp'])
    try:
        anchor = Decimal(str(raw['frame_timestamp']))
    except ArithmeticError as error:
        raise ValueError('invalid SDK global timestamp') from error
    if not anchor.is_finite() or anchor <= 0:
        raise ValueError('invalid SDK global timestamp')
    delta = (sensor - hardware + 2**31) % 2**32 - 2**31
    return round(anchor * 1_000_000) + delta * 1000


def expected_parameters(camera):
    """Public requested settings; advanced overrides are deliberately detectable."""
    result = {'enable_sync': camera['sync_rgb_depth'], 'enable_color': True, 'enable_depth': True,
              'depth_module.global_time_enabled': True}
    for prefix, stem in [('depth_module', 'depth')] + ([] if camera['device_type'].lower() == 'd405' else [('rgb_camera', 'color')]):
        automatic = camera[stem + '_auto_exposure']
        result[prefix + '.enable_auto_exposure'] = automatic
        result[prefix + '.global_time_enabled'] = True
        if automatic and stem == 'depth':
            result.update({prefix + '.auto_exposure_limit': camera['depth_auto_exposure_limit_us'],
                           prefix + '.auto_gain_limit': camera['depth_auto_gain_limit'],
                           prefix + '.auto_exposure_limit_toggle': True,
                           prefix + '.auto_gain_limit_toggle': True})
        elif not automatic:
            exposure = camera[stem + '_exposure_us']
            result[prefix + '.exposure'] = (rgb_exposure_parameter(camera['device_type'], exposure)
                                             if stem == 'color' else exposure)
            result[prefix + '.gain'] = camera[stem + '_gain']
    return result


class Metric:
    def __init__(self):
        self.values = deque(maxlen=2048)
        self.count = 0
        self.minimum = self.maximum = None
        self.total = 0.

    def add(self, value):
        value = finite(value)
        self.values.append(value); self.count += 1; self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def report(self):
        ordered = sorted(self.values)
        return {'samples': self.count, 'min': self.minimum, 'max': self.maximum,
                'mean': self.total / self.count if self.count else None,
                'p95_recent': ordered[max(0, math.ceil(.95 * len(ordered)) - 1)] if ordered else None}


class CameraCheck:
    def __init__(self, camera, *, verify_images=False, exercise_auto=False, max_age_ms=500., min_rate_ratio=.8,
                 require_equal_exposure=False):
        self.camera = camera
        self.verify_images, self.exercise_auto = verify_images, exercise_auto
        self.require_equal_exposure = require_equal_exposure
        self.max_age_ms, self.min_rate_ratio = max_age_ms, min_rate_ratio
        self.counts, self.failures, self.unknowns = Counter(), Counter(), Counter()
        self.metrics = {}
        self.pending, self.image_pending = OrderedDict(), OrderedDict()
        self.last_counter, self.last_capture = {}, {}
        self.last_received = {}
        self.last_epoch = None
        self.trace = deque(maxlen=10000)

    def metric(self, name, value):
        self.metrics.setdefault(name, Metric()).add(value)

    def fail(self, name):
        self.failures[name] += 1

    def _bounded(self, collection):
        while len(collection) > 256:
            collection.popitem(last=False)

    def raw(self, stream, raw, header_ns, receive_ns, elapsed):
        self.counts[stream] += 1; self.last_received[stream] = elapsed
        try:
            counter = int(raw['frame_number'])
            if stream in self.last_counter:
                delta = counter - self.last_counter[stream]
                if delta <= 0: self.fail(stream + ':帧号重复或倒退')
                if delta > 1: self.counts[stream + '_gaps'] += delta - 1
            self.last_counter[stream] = counter
        except (KeyError, ValueError, TypeError):
            self.unknowns[stream + ':缺少有效帧号'] += 1
        stem = 'depth' if stream == 'depth' or self.camera['device_type'].lower() == 'd405' else 'color'
        automatic = self.camera[stem + '_auto_exposure']
        for key, suffix in [('actual_exposure', 'exposure_us'), ('gain_level', 'gain')]:
            try:
                value = finite(raw[key])
                if value < 0 or (key == 'actual_exposure' and value == 0):
                    raise ValueError('invalid exposure/gain')
                self.metric(stream + '_' + suffix, value)
                if key == 'actual_exposure':
                    if value > self.camera['max_actual_exposure_us']:
                        self.fail(stream + ':实际曝光超过要求上限')
                    if automatic and stem == 'depth' and value > self.camera['depth_auto_exposure_limit_us']:
                        self.fail(stream + ':实际曝光超过配置的自动曝光限制')
                    if not automatic and abs(value - self.camera[stem + '_exposure_us']) > 100:
                        self.fail(stream + ':实际曝光与手动设定相差超过100us')
                elif automatic and stem == 'depth' and value > self.camera['depth_auto_gain_limit']:
                    self.fail(stream + ':增益超过自动增益限制')
            except (KeyError, ValueError, TypeError, OverflowError):
                self.unknowns[stream + ':缺少有效' + key] += 1
        try:
            mode = finite(raw['auto_exposure'])
            if bool(mode) != automatic: self.fail(stream + ':实际自动曝光状态与配置不符')
        except (KeyError, ValueError, TypeError, OverflowError):
            self.unknowns[stream + ':缺少自动曝光状态'] += 1
        try:
            capture = mapped_midpoint_ns(raw)
            if stream in self.last_capture and capture <= self.last_capture[stream]:
                self.fail(stream + ':映射时间重复或倒退')
            self.last_capture[stream] = capture
            age_ms = (receive_ns - capture) / 1e6
            self.metric(stream + '_receive_age_ms', age_ms)
            if age_ms < -10 or age_ms > self.max_age_ms:
                self.fail(stream + ':采集时间与接收时间间隔异常')
            group = self.pending.setdefault(header_ns, {})
            group[stream] = {'capture': capture, 'raw': raw}
            self._join(header_ns)
            self._bounded(self.pending)
        except (KeyError, ValueError, TypeError, OverflowError):
            self.fail(stream + ':缺少GLOBAL_TIME曝光中点映射依据')

    def normalized(self, data, header_ns, elapsed):
        self.counts['normalized'] += 1; self.last_received['normalized'] = elapsed
        try:
            if data['source_id'] != self.camera['id']:
                self.fail('标准化消息的相机ID不符')
            epoch = int(data['clock_epoch'])
            if self.last_epoch is not None and epoch != self.last_epoch: self.fail('检测到时钟epoch变化')
            self.last_epoch = epoch
            if int(data['capture_time_ns']) != header_ns: self.fail('元数据Header与capture_time不一致')
            key = int(data['driver_header_timestamp_ns'])
            self.pending.setdefault(key, {})['normalized'] = data
            self._join(key); self._bounded(self.pending)
            if self.verify_images:
                self.image_pending.setdefault(header_ns, {})['metadata'] = data
                self._join_image(header_ns); self._bounded(self.image_pending)
        except (KeyError, ValueError, TypeError, OverflowError):
            self.fail('标准化元数据格式不完整')

    def image(self, header_ns, rgb_ns, depth_ns, elapsed):
        self.counts['images'] += 1; self.last_received['images'] = elapsed
        self.image_pending.setdefault(header_ns, {})['image'] = (rgb_ns, depth_ns)
        self._join_image(header_ns); self._bounded(self.image_pending)

    def _join_image(self, key):
        group = self.image_pending[key]
        if not {'image', 'metadata'} <= group.keys(): return
        try:
            expected = tuple(int(group['metadata'][stream]['capture_time_ns']) for stream in ('rgb', 'depth'))
            if group['image'] != expected: self.fail('RGBD图像Header与曝光中点不一致')
            self.counts['verified_images'] += 1
        except (KeyError, ValueError, TypeError):
            self.fail('图像校验缺少曝光中点')
        del self.image_pending[key]

    def _join(self, key):
        group = self.pending[key]
        if not {'rgb', 'depth'} <= group.keys(): return
        if not group.get('paired'):
            skew = abs(group['rgb']['capture'] - group['depth']['capture']) / 1e6
            self.metric('rgb_depth_midpoint_skew_ms', skew)
            self.counts['raw_pairs'] += 1; group['paired'] = True
            if skew > self.camera['rgbd_max_midpoint_skew_ms']:
                self.fail('RGB与深度曝光中点差超过配置阈值')
            exposures = {}
            for stream in ('rgb', 'depth'):
                try:
                    value = finite(group[stream]['raw']['actual_exposure'])
                    if value <= 0:
                        raise ValueError('invalid actual exposure')
                    exposures[stream] = value
                except (KeyError, ValueError, TypeError, OverflowError):
                    exposures[stream] = None
            exposure_difference = None
            if all(value is not None for value in exposures.values()):
                exposure_difference = abs(exposures['rgb'] - exposures['depth'])
                self.metric('rgb_depth_exposure_difference_us', exposure_difference)
                self.counts['exposure_pairs'] += 1
                if self.require_equal_exposure and exposure_difference != 0:
                    self.fail('RGB与深度实际曝光时长不一致')
            elif self.require_equal_exposure:
                self.unknowns['RGB-D配对缺少有效实际曝光，无法确认时长一致'] += 1
            self.trace.append({'rgb_time_ns': group['rgb']['capture'], 'depth_time_ns': group['depth']['capture'],
                               'skew_ms': skew, 'rgb_exposure_us': exposures['rgb'],
                               'depth_exposure_us': exposures['depth'], 'exposure_difference_us': exposure_difference})
        if 'normalized' not in group: return
        try:
            for stream in ('rgb', 'depth'):
                produced = int(group['normalized'][stream]['capture_time_ns'])
                residual = abs(produced - group[stream]['capture'])
                self.metric('mapping_residual_ns', residual)
                if residual > 1000: self.fail('发布中点与原始元数据换算不一致')
                if int(group['normalized'][stream]['source_seq']) != int(group[stream]['raw']['frame_number']):
                    self.fail('RGBD来源帧号不符')
            self.counts['verified_pairs'] += 1
        except (KeyError, ValueError, TypeError, OverflowError):
            self.fail('标准化消息缺少映射结果')
        del self.pending[key]

    def report(self, duration, parameters=None):
        failures, unknowns = dict(self.failures), dict(self.unknowns)
        for stream in ('rgb', 'depth', 'normalized') + (('images',) if self.verify_images else ()):
            if self.counts[stream] < 2:
                failures[stream + ':没有足够数据'] = self.counts[stream]
            elif self.counts[stream] / duration < self.camera['fps'] * self.min_rate_ratio:
                failures[stream + ':接收帧率低于配置的80%'] = self.counts[stream]
            if duration - self.last_received.get(stream, -1) > max(.5, 3 / self.camera['fps']):
                failures[stream + ':结束时数据已中断'] = 1
        for stream in ('rgb', 'depth'):
            if self.counts[stream + '_gaps']: failures[stream + ':观测帧号缺口'] = self.counts[stream + '_gaps']
        pair_count = min(self.counts['rgb'], self.counts['depth'])
        if pair_count and self.counts['verified_pairs'] < pair_count * .95:
            failures['原始到标准化配对覆盖率低于95%'] = self.counts['verified_pairs']
        if self.verify_images and self.counts['verified_images'] < max(1, self.counts['normalized'] * .95):
            failures['标准化图像校验覆盖率低于95%'] = self.counts['verified_images']
        if parameters is None:
            unknowns['驱动参数服务未返回，未确认参数生效'] = 1
        else:
            for name, expected in expected_parameters(self.camera).items():
                if name not in parameters or parameters[name] is None:
                    unknowns['缺少驱动参数:' + name] = 1
                elif parameters[name] != expected:
                    failures['驱动参数不符:' + name] = {'expected': expected, 'actual': parameters[name]}
            if parameters.get('use_sim_time') is not False:
                failures['需要ROS系统时间，use_sim_time必须为false'] = 1
        if self.exercise_auto:
            for stream in ('rgb', 'depth'):
                stem = 'depth' if stream == 'depth' or self.camera['device_type'].lower() == 'd405' else 'color'
                if not self.camera[stem + '_auto_exposure']: continue
                values = [self.metrics.get(stream + suffix) for suffix in ('_exposure_us', '_gain')]
                if not any(m and m.count and m.maximum > m.minimum for m in values):
                    unknowns[stream + ':未观察到曝光或增益响应，请改变光照后复测'] = 1
        return {'id': self.camera['id'], 'device_type': self.camera['device_type'],
                'serial_no': self.camera['serial_no'], 'status': 'FAIL' if failures else 'UNKNOWN' if unknowns else 'PASS',
                'counts': dict(self.counts), 'fps': {s: self.counts[s] / duration for s in ('rgb', 'depth', 'normalized')},
                'metrics': {k: v.report() for k, v in self.metrics.items()},
                'failures': failures, 'unknowns': unknowns, 'parameters': parameters,
                'require_equal_exposure': self.require_equal_exposure,
                'limits': {'exposure_us': self.camera['max_actual_exposure_us'],
                           'rgb_depth_exposure_difference_us': 0 if self.require_equal_exposure else None,
                           'midpoint_skew_ms': self.camera['rgbd_max_midpoint_skew_ms'],
                           'receive_age_ms': self.max_age_ms},
                'physical_clock_accuracy_verified': False, 'trace': list(self.trace)}
