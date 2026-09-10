"""Publisher defaults must use the names declared by the official driver."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters


@pytest.mark.parametrize('model', ['d405', 'd435', 'd455'])
def test_robot_launch_sets_real_driver_qos_names_and_checks_readback(model):
    path = Path(__file__).resolve().parents[1] / 'launch/realsense_camera.launch.py'
    spec = importlib.util.spec_from_file_location('driver_qos_launch', path)
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    camera = validate_cameras([{'id': 'front', 'device_type': model, 'serial_no': 'TEST_ONLY'}])[0]
    params = evaluate_parameters(LaunchContext(), normalize_parameters([launch._parameters(camera)]))[0]
    # Unlike exposure/profile options, stream QoS options are root-level.
    keys = ('depth_qos', 'color_qos', 'depth_info_qos', 'color_info_qos')
    assert all(params[key] == 'DEFAULT' for key in keys)
    assert not any(key.endswith('_qos') and '.' in key for key in params)
    old_runtime = {**expected_parameters(camera), 'depth_qos': 'SYSTEM_DEFAULT', 'use_sim_time': False}
    result = CameraCheck(camera).report(1, old_runtime)
    assert result['failures']['驱动参数不符:depth_qos'] == {'expected': 'DEFAULT', 'actual': 'SYSTEM_DEFAULT'}
    explicit = {**camera, 'parameters': {**camera['parameters'], 'color_qos': 'SENSOR_DATA'}}
    assert launch._parameters(explicit)['color_qos'] == 'SENSOR_DATA'
    assert expected_parameters(explicit)['color_qos'] == 'SENSOR_DATA'
