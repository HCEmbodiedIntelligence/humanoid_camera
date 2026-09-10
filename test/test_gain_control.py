"""Synthetic brightness and a ROS fake driver validate gain-only feedback."""
import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.gain_control import image_brightness, next_gain


@pytest.mark.parametrize('encoding,pixel,expected', [
    ('mono8', [80], 80), ('rgb8', [255, 0, 0], 255 * 77 / 256),
    ('bgr8', [0, 0, 255], 255 * 77 / 256), ('rgba8', [0, 255, 0, 0], 255 * 150 / 256)])
def test_brightness_respects_color_order_and_row_padding(encoding, pixel, expected):
    frame = SimpleNamespace(encoding=encoding, width=1, height=2, step=len(pixel) + 2,
                            data=bytes((pixel + [255, 255]) * 2))
    assert image_brightness(frame) == expected
    frame.data = b''
    with pytest.raises(ValueError):
        image_brightness(frame)


def test_gain_feedback_has_independent_direction_deadband_steps_and_limits():
    assert next_gain(64, 10, 16, 128) == 68
    assert next_gain(64, 240, 0, 128) == 60
    assert next_gain(64, 130, 16, 128) == 64
    assert next_gain(128, 10, 16, 128) == 128
    assert next_gain(16, 240, 16, 128) == 16
    assert next_gain(120, 10, 16, 125, 4) == 124
    assert next_gain(200, 130, 16, 128) == 128
    with pytest.raises(ValueError):
        next_gain(64, 10, 16, 10)


def test_legacy_d435_loads_paired_manual_exposure_without_mutating_d405():
    cameras = validate_cameras([
        {'id': 'head', 'device_type': 'd435', 'serial_no': '435',
         'depth_auto_exposure': True, 'depth_exposure_us': 4500, 'color_exposure_us': 4500,
         'sync_rgb_depth': False},
        {'id': 'left', 'device_type': 'd405', 'serial_no': '405'}])
    head, left = cameras
    assert head['depth_exposure_us'] == head['color_exposure_us'] == 3900
    assert head['depth_auto_exposure'] is head['color_auto_exposure'] is False
    assert head['depth_auto_gain'] is head['color_auto_gain'] is True
    assert head['sync_rgb_depth'] is True
    assert left['depth_auto_exposure'] is True
    assert left['depth_auto_gain'] is False


def test_feedback_uses_driver_ranges_writes_only_gains_and_pauses_on_exposure_change():
    path = Path(__file__).resolve().parents[1] / 'scripts/gain_controller.py'
    spec = importlib.util.spec_from_file_location('gain_controller_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'driver_prefix:=/gain_test/camera'], domain_id=172)
    driver = rclpy.create_node('camera', namespace='/gain_test', use_global_arguments=False)
    writes = []
    for stem, exposure, minimum in [('depth_module', 3900, 16), ('rgb_camera', 39, 0)]:
        driver.declare_parameter(stem + '.enable_auto_exposure', False)
        driver.declare_parameter(stem + '.exposure', exposure)
        driver.declare_parameter(stem + '.gain', 64, ParameterDescriptor(
            integer_range=[IntegerRange(from_value=minimum, to_value=128, step=1)]))

    def changed(parameters):
        writes.extend((parameter.name, parameter.value) for parameter in parameters)
        return SetParametersResult(successful=True)

    driver.add_on_set_parameters_callback(changed)
    controller = module.GainController()
    publishers = [driver.create_publisher(Image, '/gain_test/camera/' + suffix, qos_profile_sensor_data)
                  for suffix in ('infra1/image_rect_raw', 'color/image_raw')]
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(controller)

    def feed(duration, stop_when=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for publisher, value in zip(publishers, (10, 240)):
                image = Image(height=2, width=2, encoding='mono8', step=2, data=bytes([value] * 4))
                image.header.stamp = driver.get_clock().now().to_msg()
                publisher.publish(image)
            executor.spin_once(timeout_sec=.02)
            if stop_when and stop_when():
                return

    try:
        feed(6., lambda: len(writes) >= 2)
        assert driver.get_parameter('depth_module.gain').value > 64
        assert driver.get_parameter('rgb_camera.gain').value < 64
        assert all(name in ('depth_module.gain', 'rgb_camera.gain') for name, _ in writes)
        assert driver.get_parameter('depth_module.exposure').value == 3900
        assert driver.get_parameter('rgb_camera.exposure').value == 39
        driver.set_parameters([Parameter('rgb_camera.enable_auto_exposure', value=True)])
        feed(2., lambda: '曝光模式' in controller.status_text)
        count = len(writes)
        feed(.6)
        assert len(writes) == count
        assert '曝光模式' in controller.status_text
    finally:
        executor.shutdown()
        controller.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
