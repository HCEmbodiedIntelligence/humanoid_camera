"""Bounded association after independent driver publication; no SDK syncer.

Original packets are archived separately by the recorder, before this association.
Missing counterparts never consume a different frame beyond the configured limit.
"""
from collections import OrderedDict


class ExposurePairs:
    def __init__(self, max_skew_ns, max_bytes, diagnostics):
        self.max_skew_ns, self.max_bytes = max_skew_ns, max_bytes
        self.diagnostics = diagnostics
        self.pending = OrderedDict()
        self.bytes = 0

    @staticmethod
    def parts(name, packet):
        return {name: packet.image, name + '_meta': packet,
                name + '_info': packet.camera_info}

    def clear(self, reason):
        for (name, _), (packet, _) in self.pending.items():
            self.diagnostics.discard(reason, self.parts(name, packet))
        self.pending.clear()
        self.bytes = 0

    def add(self, name, packet, stamp):
        key = (name, stamp)
        if key in self.pending:
            return None
        size = len(packet.image.data) + len(packet.json_data.encode())
        if size > self.max_bytes:
            self.diagnostics.discard('oversize_' + name, self.parts(name, packet))
            return None
        matches = [candidate for candidate in self.pending
                   if candidate[0] != name and abs(candidate[1] - stamp) <= self.max_skew_ns]
        if matches:
            closest = min(matches, key=lambda item: (abs(item[1] - stamp), item[1]))
            other, used = self.pending.pop(closest)
            self.bytes -= used
            return {**self.parts(name, packet), **self.parts(closest[0], other)}
        while self.pending and (self.bytes + size > self.max_bytes or len(self.pending) >= 32):
            old_key, (old, used) = self.pending.popitem(last=False)
            self.bytes -= used
            self.diagnostics.discard('independent_pair_buffer_limit', self.parts(old_key[0], old))
        self.pending[key] = (packet, size)
        self.bytes += size
        return None
