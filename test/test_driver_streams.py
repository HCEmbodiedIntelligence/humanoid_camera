"""Prevent implicit IR streams without dropping the D435 brightness feedback."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters


@pytest.fixture
def parameters():
    path = Path(__file__).resolve().parents[1] / 'launch/realsense_camera.launch.py'
    spec = importlib.util.spec_from_file_location('infrared_launch', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return lambda camera: evaluate_parameters(LaunchContext(), normalize_parameters([module._parameters(camera)]))[0]


@pytest.mark.parametrize('model,infra1', [('d405', False), ('d435', True), ('d435i', True), ('d455', False)])
def test_only_required_streams_are_enabled(parameters, model, infra1):
    camera = validate_cameras([{'id': 'front', 'device_type': model}])[0]
    values = parameters(camera)
    assert values['enable_infra1'] is infra1 and values['enable_infra2'] is False
    assert values['enable_color'] is values['enable_depth'] is True
    assert values['depth_module.depth_profile'] == '640,480,30'
    assert values['spatial_filter.enable'] is True and values['temporal_filter.enable'] is False
    expected = expected_parameters(camera)
    assert values['enable_infra1'] == expected['enable_infra1']
    old_runtime = {**expected, 'enable_infra2': True, 'use_sim_time': False}
    assert '驱动参数不符:enable_infra2' in CameraCheck(camera).report(1, old_runtime)['failures']


def test_explicit_ir_output_is_preserved_and_feedback_is_required(parameters):
    camera = validate_cameras([{'id': 'front', 'device_type': 'd435',
                               'parameters': {'enable_infra1': False, 'enable_infra2': True}}])[0]
    assert parameters(camera)['enable_infra1'] is True
    assert parameters(camera)['enable_infra2'] is True
    camera['depth_auto_gain'] = False
    assert parameters(camera)['enable_infra1'] is False
    assert expected_parameters(camera)['enable_infra2'] is True


@pytest.mark.parametrize('filename', ['d435-humble.yaml', 'd405-humble-auto-brightness.yaml'])
def test_standalone_default_does_not_open_extra_ir(filename):
    values = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config' / filename).read_text())['/**']['ros__parameters']
    assert values['enable_infra1'] is values['enable_infra2'] is False
