"""Bounded callback evidence; metadata reception gaps are not missing photos."""
from collections import deque
import math


def integer(value):
    return value if type(value) is int and value >= 0 else None


def number(value):
    try:
        return value if type(value) in (int, float) and math.isfinite(value) else None
    except OverflowError:
        return None


class FrameTrace:
    def __init__(self, source_id, limit=20000):
        self.source_id = source_id
        self.events = deque(maxlen=limit)
        self.discarded = 0

    def append(self, kind, elapsed, receive_ns, **fields):
        if len(self.events) == self.events.maxlen:
            self.discarded += 1
        self.events.append({'kind': kind, 'elapsed_sec': number(elapsed),
                            'receive_ns': integer(receive_ns), **fields})

    def raw(self, stream, raw, header_ns, receive_ns, elapsed):
        self.append('raw_' + stream, elapsed, receive_ns,
                    header_ns=integer(header_ns), frame_number=integer(raw.get('frame_number')),
                    sdk_frame_timestamp_ms=number(raw.get('frame_timestamp')),
                    sdk_arrival_ms=number(raw.get('time_of_arrival')),
                    backend_timestamp_ms=number(raw.get('backend_timestamp')),
                    raw_actual_exposure=number(raw.get('actual_exposure')),
                    gain_level=number(raw.get('gain_level')))

    def normalized(self, data, header_ns, receive_ns, elapsed):
        if data.get('source_id') != self.source_id:
            return
        fields = {'header_ns': integer(header_ns),
                  'adapter_group_ns': integer(data.get('receive_time_ns')),
                  'clock_epoch': integer(data.get('clock_epoch')),
                  'clock_id': str(data.get('clock_id', ''))[:128]}
        for stream in ('rgb', 'depth'):
            part = data.get(stream)
            if not isinstance(part, dict):
                return
            raw = part.get('raw_metadata')
            if not isinstance(raw, dict):
                return
            counter = integer(raw.get('frame_number'))
            # Require source identity and mapped Header agreement, not just a
            # coincidentally equal frame counter from unrelated metadata.
            if counter is None or counter != integer(part.get('source_seq')):
                return
            fields[stream + '_frame_number'] = counter
            fields[stream + '_header_ns'] = integer(part.get('capture_time_ns'))
            fields[stream + '_sdk_arrival_ms'] = number(raw.get('time_of_arrival'))
        if fields['header_ns'] is None or fields['header_ns'] != fields['rgb_header_ns']:
            return
        self.append('normalized', elapsed, receive_ns, **fields)

    def image(self, header_ns, rgb_ns, depth_ns, receive_ns, elapsed):
        self.append('image', elapsed, receive_ns, header_ns=integer(header_ns),
                    rgb_header_ns=integer(rgb_ns), depth_header_ns=integer(depth_ns))

    def report(self):
        events = list(self.events)
        result = {'schema_version': 1, 'events': events, 'discarded_events': self.discarded,
                  'max_events': self.events.maxlen, 'gap_evidence': {}}
        # A partial ring buffer cannot establish whether an earlier callback
        # was absent. Preserve its events, but never infer losses from it.
        if self.discarded:
            result['classification_status'] = 'trace_truncated'
            return result
        normalized = [e for e in events if e['kind'] == 'normalized']
        epochs = {(e['clock_id'], e['clock_epoch']) for e in normalized}
        if len(epochs) > 1:
            result['classification_status'] = 'clock_changed'
            return result
        images = {(e['header_ns'], e['rgb_header_ns'], e['depth_header_ns'])
                  for e in events if e['kind'] == 'image'
                  and all(e[k] is not None for k in ('header_ns', 'rgb_header_ns', 'depth_header_ns'))}
        joined = [e for e in normalized
                  if (e['header_ns'], e['rgb_header_ns'], e['depth_header_ns']) in images]
        result['classification_status'] = 'observed_callbacks_only'
        for stream in ('rgb', 'depth'):
            raw = [e['frame_number'] for e in events if e['kind'] == 'raw_' + stream]
            if len(raw) < 2 or any(n is None for n in raw):
                result['gap_evidence'][stream] = {'status': 'insufficient_frame_numbers'}
                continue
            if any(b <= a for a, b in zip(raw, raw[1:])):
                result['gap_evidence'][stream] = {'status': 'nonmonotonic_frame_numbers'}
                continue
            seen = set(raw)
            # Compare strictly inside the raw receiver's observed frame span;
            # warm-up/backlog or end-of-window differences are not losses.
            recovered = sorted({e[stream + '_frame_number'] for e in joined
                                if raw[0] < e[stream + '_frame_number'] < raw[-1]} - seen)
            gap_count = raw[-1] - raw[0] + 1 - len(raw)
            result['gap_evidence'][stream] = {
                'status': 'observed_callbacks_only', 'raw_gap_count': gap_count,
                'present_in_normalized_images': len(recovered),
                'recovered_frame_numbers': recovered,
                'unresolved_gap_count': gap_count - len(recovered)}
        return result
