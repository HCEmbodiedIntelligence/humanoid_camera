"""Small counters for locating loss before and after RGBD assembly."""
from collections import Counter
import time

INPUTS = ('rgb', 'depth', 'rgb_meta', 'depth_meta', 'cloud')
REQUIRED = INPUTS[:4]


class PipelineDiagnostics:
    def __init__(self, source_id, instance_id):
        self.source_id, self.instance_id = source_id, instance_id
        self.received, self.discarded, self.missing = Counter(), Counter(), Counter()
        self.last_received = {}
        self.published = 0
        self.started = time.monotonic()

    def receive(self, name):
        self.received[name] += 1
        self.last_received[name] = time.monotonic()

    def discard(self, reason, parts):
        self.discarded[reason] += 1
        # A late cloud by itself is not an attempted RGBD pair.
        if any(name in parts for name in REQUIRED):
            for name in REQUIRED:
                if name not in parts:
                    self.missing[name] += 1

    def snapshot(self, *, pending_groups, pending_bytes, clock_epoch):
        now = time.monotonic()
        return {'schema_version': 1, 'source_id': self.source_id, 'instance_id': self.instance_id,
                'uptime_sec': now - self.started, 'received': {name: self.received[name] for name in INPUTS},
                'published_pairs': self.published, 'discarded_groups': dict(self.discarded),
                'missing_when_discarded': dict(self.missing), 'clock_epoch': clock_epoch,
                'last_receive_age_sec': {name: now - self.last_received[name] if name in self.last_received else None
                                         for name in INPUTS},
                'pending_groups': pending_groups, 'pending_bytes': pending_bytes}


def summarize_window(first, last):
    """Use one producer's monotonic window; never mix it with receiver counts."""
    if not first or not last or first.get('instance_id') != last.get('instance_id'):
        return None
    elapsed = float(last['uptime_sec']) - float(first['uptime_sec'])
    if elapsed <= 0:
        return None
    pairs = int(last['published_pairs']) - int(first['published_pairs'])
    received = {name: int(last['received'][name]) - int(first['received'][name]) for name in INPUTS}
    if pairs < 0 or any(count < 0 for count in received.values()):
        return None
    return {'duration_sec': elapsed, 'received_hz': {name: count / elapsed for name, count in received.items()},
            'published_pairs_hz': pairs / elapsed,
            'discarded_groups': {name: count - first.get('discarded_groups', {}).get(name, 0)
                                 for name, count in last.get('discarded_groups', {}).items()},
            'missing_when_discarded': {name: count - first.get('missing_when_discarded', {}).get(name, 0)
                                       for name, count in last.get('missing_when_discarded', {}).items()}}
