"""Check QoS negotiation and real full-size ROS image transport without hardware."""
import importlib.util
import json
from pathlib import Path
import time
from types import SimpleNamespace as Obj
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from humanoid_camera.transport import CaptureSubscription, capture_qos


def test_capture_prefers_reliable_and_handles_discovery_restart_and_raw_mode():
    created, destroyed = [], []
    class Node:
        offered = []
        def create_subscription(self, kind, topic, callback, qos, **kwargs):
            sub = Obj(topic_name=topic, callback=callback, qos=qos, kwargs=kwargs)
            created.append(sub)
            return sub
        def destroy_subscription(self, sub): destroyed.append(sub)
        def get_publishers_info_by_topic(self, topic): return self.offered
        def get_logger(self): return Obj(info=lambda text: None)
    node = Node()
    callback = lambda message: message
    receiver = CaptureSubscription(node, object, '/image', callback, raw=True)
    receiver.refresh()
    assert len(created) == 1 and receiver.reliability == ReliabilityPolicy.RELIABLE
    assert created[-1].qos.durability == DurabilityPolicy.VOLATILE
    for policy in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
        node.offered = [Obj(qos_profile=capture_qos(reliability=policy))]
        receiver.refresh()
        assert created[-1].qos.reliability == policy
        assert created[-1].kwargs == {'raw': True} and created[-1].callback is callback
    node.offered = []
    receiver.refresh()
    assert receiver.reliability == ReliabilityPolicy.RELIABLE
    receiver.close()
    assert len(created) == len(destroyed) == 3


@pytest.mark.parametrize('offered', [ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT])
def test_full_size_rgbd_preserves_pixels_and_timestamps_and_cloud_is_on_demand(offered):
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image, PointCloud2
    from realsense2_camera_msgs.msg import Metadata, RGBD
    from std_msgs.msg import Header
    path = Path(__file__).resolve().parents[1] / 'scripts/timestamp_adapter.py'
    spec = importlib.util.spec_from_file_location('full_size_transport_adapter', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    namespace = '/capture_' + uuid.uuid4().hex[:10]
    rclpy.init(args=['--ros-args', '-r', '__ns:=' + namespace], domain_id=225)
    adapter = module.TimestampAdapter()
    source = rclpy.create_node('synthetic_source')
    sink = rclpy.create_node('synthetic_sink')
    executor = SingleThreadedExecutor()
    for node in (source, adapter, sink): executor.add_node(node)
    publishers = {name: source.create_publisher(kind, '/front/camera/' + suffix, capture_qos(10, offered))
                  for name, kind, suffix in [('rgb', Image, 'color/image_raw'), ('depth', Image, 'depth/image_rect_raw'),
                  ('rgb_meta', Metadata, 'color/metadata'), ('depth_meta', Metadata, 'depth/metadata')]}
    clouds = source.create_publisher(PointCloud2, '/front/camera/depth/color/points', capture_qos(2, offered))
    pairs, metadata, cloud_output = [], [], []
    sink.create_subscription(RGBD, namespace + '/normalized/rgbd', pairs.append, capture_qos())
    sink.create_subscription(Metadata, namespace + '/normalized/metadata', metadata.append, capture_qos(30))
    def until(predicate, seconds=4):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate()
    try:
        until(lambda: all(sub.reliability == offered for sub in adapter.capture_subscriptions.values())
              and all(pub.get_subscription_count() for pub in publishers.values())
              and adapter.pair_pub.get_subscription_count() > 0)
        assert adapter.cloud_subscription is None and clouds.get_subscription_count() == 0
        colors = bytes([32, 64, 128]) * (640 * 480)
        depths = bytes([0x34, 0x12]) * (640 * 480)
        count = 45; sent = 0; started = time.monotonic(); final_header = None
        while sent < count:
            now = time.monotonic()
            if now >= started + sent / 30:
                h = Header(stamp=source.get_clock().now().to_msg(), frame_id='optical')
                t = module.ns(h)
                raw = {'clock_domain':'global_time', 'frame_timestamp': f'{t//1_000_000}.{t%1_000_000:06d}',
                       'hw_timestamp':100000 + sent*33333, 'sensor_timestamp':98000 + sent*33333,
                       'frame_number':sent, 'actual_exposure':4000}
                for stream, encoding, step, payload in [('rgb','rgb8',1920,colors), ('depth','16UC1',1280,depths)]:
                    publishers[stream + '_meta'].publish(Metadata(header=h, json_data=json.dumps(raw)))
                    publishers[stream].publish(Image(header=h,width=640,height=480,encoding=encoding,step=step,data=payload))
                final_header = h
                sent += 1
            executor.spin_once(timeout_sec=.002)
        until(lambda: len(pairs) >= 40 and len(metadata) >= 40)
        assert all(bytes(pair.rgb.data) == colors and bytes(pair.depth.data) == depths for pair in pairs)
        records = {module.ns(msg.header): json.loads(msg.json_data) for msg in metadata}
        assert sum(module.ns(pair.header) in records for pair in pairs) >= 40
        for pair in pairs:
            if module.ns(pair.header) in records:
                record = records[module.ns(pair.header)]
                assert module.ns(pair.rgb.header) == module.midpoint(record['rgb']['raw_metadata'])[0]
                assert module.ns(pair.depth.header) == module.midpoint(record['depth']['raw_metadata'])[0]
        # A consumer enables cloud transfer; no cloud payload was needed for RGBD.
        cloud_sub = sink.create_subscription(PointCloud2, namespace+'/normalized/points', cloud_output.append, capture_qos(2))
        until(lambda: adapter.cloud_subscription is not None and adapter.cloud_subscription.reliability == offered
              and clouds.get_subscription_count() > 0)
        key = module.ns(final_header)
        assert key in adapter.lineage
        clouds.publish(PointCloud2(header=final_header, data=b'\x01\x02\x03\x04'))
        until(lambda: bool(cloud_output))
        assert bytes(cloud_output[0].data) == b'\x01\x02\x03\x04'
        assert module.ns(cloud_output[0].header) == adapter.lineage[key]['depth']['capture_time_ns']
        sink.destroy_subscription(cloud_sub)
        until(lambda: adapter.cloud_subscription is None and clouds.get_subscription_count() == 0)
    finally:
        executor.shutdown()
        for node in (sink, adapter, source): node.destroy_node()
        rclpy.try_shutdown()
