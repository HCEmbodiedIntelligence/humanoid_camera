#!/usr/bin/env python3
"""D435 manual exposure with independent IR/RGB gain feedback via driver services."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import String

from humanoid_camera.exposure import rgb_exposure_parameter
from humanoid_camera.gain_control import image_brightness, next_gain


class GainController(Node):
    def __init__(self):
        super().__init__('camera_gain_controller')
        defaults = {'driver_prefix': '/front/camera', 'rgb_topic': '',
                    'depth_auto_gain': True, 'color_auto_gain': True,
                    'depth_auto_gain_limit': 128, 'color_auto_gain_limit': 128,
                    'depth_exposure_us': 3900, 'color_exposure_us': 3900}
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        prefix = self.settings['driver_prefix'].rstrip('/')
        self.streams = {stem: module for stem, module in [('depth', 'depth_module'), ('color', 'rgb_camera')]
                        if self.settings[stem + '_auto_gain']}
        self.samples, self.ranges, self.last_set, self.last_sample = {}, {}, {}, {}
        self.future = self.operation = None
        self.deadline = 0.
        self.status_text = ''
        self.get_client = self.create_client(GetParameters, prefix + '/get_parameters')
        self.describe_client = self.create_client(DescribeParameters, prefix + '/describe_parameters')
        self.set_client = self.create_client(SetParameters, prefix + '/set_parameters')
        self.status_pub = self.create_publisher(String, 'gain_control/status', 1)
        for stem in self.streams:
            topic = prefix + '/infra1/image_rect_raw' if stem == 'depth' else self.settings['rgb_topic'] or prefix + '/color/image_raw'
            self.create_subscription(Image, topic, lambda msg, stem=stem: self.receive(stem, msg), qos_profile_sensor_data)
        self.create_timer(.2, self.tick)

    def status(self, state, **details):
        self.status_pub.publish(String(data=json.dumps({'state': state, **details}, ensure_ascii=False)))
        if state != self.status_text:
            self.get_logger().info(state)
            self.status_text = state

    def receive(self, stem, message):
        now = time.monotonic()
        if now - self.last_sample.get(stem, 0.) < .1:
            return
        self.last_sample[stem] = now
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not -.1 <= age <= .5 or stamp <= self.last_set.get(stem, 0) + 100_000_000:
            return
        try:
            brightness = image_brightness(message)
        except ValueError as error:
            self.status(str(error))
            return
        self.samples[stem] = (now, brightness, stamp)

    def request(self, operation, client, request):
        self.operation = operation
        self.future = client.call_async(request)
        self.deadline = time.monotonic() + 2.

    def tick(self):
        try:
            self.advance()
        except Exception as error:
            self.future = self.operation = None
            self.ranges.clear()
            self.status('自动增益暂停: ' + str(error))

    def advance(self):
        if self.future is not None:
            if not self.future.done():
                if time.monotonic() > self.deadline:
                    self.future.cancel()
                    self.future = self.operation = None
                    self.ranges.clear()
                    self.status('自动增益暂停: 驱动参数服务超时')
                return
            response, operation = self.future.result(), self.operation
            self.future = self.operation = None
            if operation == 'describe':
                if len(response.descriptors) != len(self.streams):
                    raise ValueError('驱动未返回完整增益范围')
                for stem, descriptor in zip(self.streams, response.descriptors):
                    if not descriptor.integer_range or descriptor.read_only:
                        raise ValueError(stem + ': 驱动未提供可写整数增益范围')
                    value = descriptor.integer_range[0]
                    self.ranges[stem] = (value.from_value, min(value.to_value, self.settings[stem + '_auto_gain_limit']), value.step or 1)
            elif operation == 'get':
                self.adjust(response)
                return
            elif operation == 'set':
                if len(response.results) != len(self.pending_stems) or not all(result.successful for result in response.results):
                    raise ValueError('增益设置失败: ' + '; '.join(result.reason for result in response.results if not result.successful))
                for stem in self.pending_stems:
                    self.last_set[stem] = self.get_clock().now().nanoseconds
                    self.samples.pop(stem, None)
                self.status('自动增益运行中', gains=self.pending_gains)
                return
        if not all(client.service_is_ready() for client in (self.describe_client, self.get_client, self.set_client)):
            self.ranges.clear()
            self.status('等待相机驱动参数服务')
            return
        if not self.ranges:
            names = [module + '.gain' for module in self.streams.values()]
            self.request('describe', self.describe_client, DescribeParameters.Request(names=names))
            return
        names = [module + '.' + leaf for module in self.streams.values()
                 for leaf in ('enable_auto_exposure', 'exposure', 'gain')]
        self.request('get', self.get_client, GetParameters.Request(names=names))

    def adjust(self, response):
        if len(response.values) != 3 * len(self.streams):
            raise ValueError('驱动未返回完整曝光和增益设置')
        updates, stems, gains, brightnesses = [], [], {}, {}
        for index, (stem, module) in enumerate(self.streams.items()):
            mode, exposure, gain = response.values[index * 3:index * 3 + 3]
            expected = self.settings[stem + '_exposure_us']
            if stem == 'color':
                expected = rgb_exposure_parameter('d435', expected)
            if mode.type != 1 or mode.bool_value or exposure.type != 2 or exposure.integer_value != expected or gain.type != 2:
                raise ValueError(stem + ': 曝光模式或曝光值已改变，请恢复配置后重启相机')
            sample = self.samples.get(stem)
            if sample is None or time.monotonic() - sample[0] > .5:
                continue
            brightnesses[stem] = sample[1]
            value = next_gain(gain.integer_value, sample[1], *self.ranges[stem])
            if value != gain.integer_value:
                updates.append(Parameter(module + '.gain', value=value).to_parameter_msg())
                stems.append(stem)
                gains[stem] = value
        if updates:
            self.pending_stems, self.pending_gains = stems, gains
            self.request('set', self.set_client, SetParameters.Request(parameters=updates))
        else:
            self.status('自动增益运行中' if len(brightnesses) == len(self.streams) else '等待新鲜红外/RGB亮度反馈', brightness=brightnesses)


def main():
    rclpy.init()
    node = GainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
