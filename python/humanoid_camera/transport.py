"""Prefer retransmission for capture, while accepting best-effort-only sources."""
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def capture_qos(depth=10, reliability=ReliabilityPolicy.RELIABLE):
    # Volatile avoids draining old transient-local images on startup.
    return QoSProfile(depth=depth, reliability=reliability, durability=DurabilityPolicy.VOLATILE)


class CaptureSubscription:
    def __init__(self, node, message_type, topic, callback, *, depth=10, **kwargs):
        self.node, self.message_type, self.topic, self.callback = node, message_type, topic, callback
        self.depth, self.kwargs = depth, kwargs
        self.reliability = ReliabilityPolicy.RELIABLE
        self.offered = []
        self.subscription = self._create()

    def _create(self):
        return self.node.create_subscription(self.message_type, self.topic, self.callback,
                                             capture_qos(self.depth, self.reliability), **self.kwargs)

    def refresh(self):
        topic = self.subscription.topic_name
        publishers = self.node.get_publishers_info_by_topic(topic)
        self.offered = [info.qos_profile.reliability.name for info in publishers]
        # With no publisher, keep the last choice until discovery has evidence.
        if not publishers:
            return
        selected = (ReliabilityPolicy.BEST_EFFORT if any(
            info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for info in publishers)
            else ReliabilityPolicy.RELIABLE)
        if selected != self.reliability:
            self.node.destroy_subscription(self.subscription)
            self.reliability = selected
            self.subscription = self._create()
            self.node.get_logger().info(f'{topic}: capture subscription QoS={selected.name}')

    def state(self):
        return {'requested_reliability': self.reliability.name,
                'offered_reliability': self.offered, 'history_depth': self.depth,
                'durability': 'VOLATILE'}

    def close(self):
        self.node.destroy_subscription(self.subscription)
