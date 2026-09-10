"""Saved overrides and every native launch must disable depth temporal filtering."""
import copy
import importlib.util
from pathlib import Path

import pytest
import rclpy
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters


def load_launch(filename):
    path = Path(__file__).resolve().parents[1] / 'launch' / filename
    spec = importlib.util.spec_from_file_location('depth_processing_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('advanced', [
    {'temporal_filter.enable': True},
    {'temporal_filter': {'enable': True, 'smooth_alpha': .6}},
    {'temporal_filter.enable': False, 'temporal_filter': {'enable': True}},
    {'temporal_filter': {'enable': False}, 'temporal_filter.enable': True},
])
def test_all_cameras_disable_flat_and_nested_overrides_when_saved_and_launched(advanced):
    original = [{'id': ident, 'device_type': model, 'serial_no': str(index),
                 'width': 1280, 'height': 720, 'fps': 15,
                 'align_depth': True, 'pointcloud': True,
                 'parameters': {**advanced, 'spatial_filter.enable': True,
                                'depth_module.depth_qos': 'SENSOR_DATA'}}
                for index, (ident, model) in enumerate([
                    ('camera_left', 'd405'), ('camera_right', 'd405'), ('camera_hand', 'd435')])]
    before = copy.deepcopy(original)
    saved = validate_cameras(original)
    assert original == before
    assert validate_cameras(yaml.safe_load(yaml.safe_dump(saved))) == saved
    driver_parameters = load_launch('realsense_camera.launch.py')._parameters
    for camera in saved:
        assert camera['parameters']['temporal_filter.enable'] is False
        if 'temporal_filter' in camera['parameters']:
            assert camera['parameters']['temporal_filter']['enable'] is False
        # Even an override applied after validation must not re-enable the filter.
        for parameters in (camera['parameters'], original[0]['parameters']):
            params = evaluate_parameters(LaunchContext(), normalize_parameters([
                driver_parameters({**camera, 'parameters': parameters})]))[0]
            assert params['temporal_filter.enable'] is False
            assert params['spatial_filter.enable'] is True
            assert params['depth_module.depth_qos'] == 'SENSOR_DATA'
            assert params['depth_module.depth_profile'] == '1280,720,15'
            assert params['enable_sync'] is True
            assert params['align_depth.enable'] is True
            assert params['pointcloud.enable'] is True
            assert params['depth_module.auto_exposure_limit'] == 4500
            if 'temporal_filter' in advanced and 'smooth_alpha' in advanced['temporal_filter']:
                assert params['temporal_filter.smooth_alpha'] == .6


@pytest.mark.parametrize('filename', ['d405.launch.py', 'd435.launch.py'])
def test_standalone_launch_overrides_custom_yaml_in_ros_parameter_parser(filename, tmp_path):
    custom = tmp_path / 'custom.yaml'
    custom.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'temporal_filter': {'enable': True}, 'spatial_filter.enable': True,
        'depth_module.depth_profile': '640,480,30'}}}))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(custom), namespace='front',
                                         camera_name='camera', serial_no='TEST_ONLY')
    description = load_launch(filename).generate_launch_description()
    driver = next(action for action in description.entities
                  if isinstance(action, Node) and action.node_package == 'realsense2_camera')
    # Inspect the normalized launch parameters without executing a camera process.
    values = evaluate_parameters(context, driver._Node__parameters)
    cli_args = ['--ros-args']
    for index, value in enumerate(values):
        if isinstance(value, dict):
            path = tmp_path / f'launch-{index}.yaml'
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': value}}))
        else:
            path = value
        cli_args.extend(['--params-file', str(path)])
    ros_context = rclpy.Context()
    rclpy.init(context=ros_context, domain_id=171)
    node = None
    try:
        node = rclpy.create_node('camera', namespace='/front', context=ros_context,
                                 cli_args=cli_args, use_global_arguments=False,
                                 automatically_declare_parameters_from_overrides=True)
        assert node.get_parameter('temporal_filter.enable').value is False
        assert node.get_parameter('spatial_filter.enable').value is True
        assert node.get_parameter('depth_module.depth_profile').value == '640,480,30'
    finally:
        if node is not None:
            node.destroy_node()
        ros_context.shutdown()


@pytest.mark.parametrize('value', [True, None])
def test_acceptance_detects_enabled_or_unknown_temporal_filter(value):
    camera = validate_cameras([{'id': 'front', 'device_type': 'd405'}])[0]
    parameters = {**expected_parameters(camera), 'use_sim_time': False,
                  'temporal_filter.enable': value}
    result = CameraCheck(camera).report(1., parameters)
    if value is True:
        assert result['failures']['驱动参数不符:temporal_filter.enable'] == {
            'expected': False, 'actual': True}
    else:
        assert result['unknowns']['缺少驱动参数:temporal_filter.enable'] == 1
