"""D435 exposure values must reach the vendor in UVC units, before streaming."""
import importlib.util
from pathlib import Path

import pytest
import yaml

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.exposure import rgb_exposure_parameter
from humanoid_camera.health_check import expected_parameters
from humanoid_manager.deployment import DeploymentError


@pytest.fixture
def driver_parameters():
    path = Path(__file__).resolve().parents[1] / 'launch/realsense_camera.launch.py'
    spec = importlib.util.spec_from_file_location('exposure_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._parameters


@pytest.mark.parametrize('model', ['d435', 'D435', 'd435i', 'd435f', 'd435if'])
@pytest.mark.parametrize('microseconds,ticks', [(100, 1), (3900, 39), (4500, 45), (4599, 45), (5000, 50)])
def test_rgb_exposure_uses_uvc_units_without_rounding_above_limit(driver_parameters, model, microseconds, ticks):
    camera = validate_cameras([{'id': 'head', 'device_type': model,
        'color_exposure_us': microseconds}])[0]
    params = driver_parameters(camera)
    assert params['rgb_camera.exposure'] == ticks
    assert ticks * 100 <= microseconds <= 5000
    assert expected_parameters(camera)['rgb_camera.exposure'] == ticks
    assert params['depth_module.enable_auto_exposure'] is False
    assert params['depth_module.exposure'] == 3900


def test_legacy_auto_and_advanced_overrides_cannot_bypass_rgb_policy(driver_parameters):
    camera = validate_cameras([{'id': 'head', 'device_type': 'd435', 'color_auto_exposure': True,
        'color_gain': 72, 'parameters': {'rgb_camera.enable_auto_exposure': True,
            'rgb_camera.exposure': 4500, 'rgb_camera.gain': 128, 'rgb_camera.brightness': 10}}])[0]
    assert camera['color_auto_exposure'] is False
    assert camera['parameters'] == {'rgb_camera.brightness': 10, 'temporal_filter.enable': False, 'spatial_filter.enable': True}
    params = driver_parameters(camera)
    assert params['rgb_camera.enable_auto_exposure'] is False
    assert params['rgb_camera.exposure'] == 39
    assert params['rgb_camera.gain'] == 72
    assert params['rgb_camera.brightness'] == 10
    assert params['depth_module.enable_auto_exposure'] is False


@pytest.mark.parametrize('microseconds', [0, 99, 5001])
def test_invalid_manual_duration_is_rejected_even_in_legacy_auto_configuration(microseconds):
    with pytest.raises(DeploymentError, match='曝光|color_exposure_us'):
        validate_cameras([{'id': 'head', 'device_type': 'd435', 'color_auto_exposure': True,
                          'color_exposure_us': microseconds}])


def test_d405_shared_exposure_stays_in_microseconds(driver_parameters):
    camera = validate_cameras([{'id': 'left', 'device_type': 'd405',
        'depth_auto_exposure': False, 'depth_exposure_us': 4500}])[0]
    params = driver_parameters(camera)
    assert params['depth_module.exposure'] == 4500
    assert 'rgb_camera.exposure' not in params
    assert rgb_exposure_parameter('d405', 4500) == 4500


def test_manual_policy_preserves_video_profile_and_processing_parameters(driver_parameters):
    camera = validate_cameras([{'id': 'head', 'device_type': 'd435', 'width': 1280,
        'height': 720, 'fps': 15, 'align_depth': True, 'pointcloud': True,
        'parameters': {'color_qos': 'SENSOR_DATA'}}])[0]
    params = driver_parameters(camera)
    assert params['rgb_camera.color_profile'] == params['depth_module.depth_profile'] == '1280,720,15'
    assert params['color_qos'] == 'SENSOR_DATA'
    assert params['enable_sync'] is True
    assert params['align_depth.enable'] is True
    assert params['pointcloud.enable'] is True
    assert params['rgb_camera.global_time_enabled'] is True


def test_d435_nested_advanced_values_cannot_override_manual_pair_or_feedback_stream(driver_parameters):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    camera = validate_cameras([{'id': 'head', 'device_type': 'd435', 'fps': 15,
        'parameters': {'depth_module': {'enable_auto_exposure': True, 'exposure': 8000,
                        'gain': 200, 'depth_profile': '640,480,90', 'infra_profile': '640,480,90', 'hdr_enabled': True},
                       'rgb_camera': {'enable_auto_exposure': True, 'exposure': 80,
                        'gain': 100, 'color_profile': '640,480,30'},
                       'enable_infra1': False, 'enable_sync': False}}])[0]
    params = evaluate_parameters(LaunchContext(), normalize_parameters([driver_parameters(camera)]))[0]
    assert params['depth_module.enable_auto_exposure'] is params['rgb_camera.enable_auto_exposure'] is False
    assert params['depth_module.exposure'] == 3900
    assert params['rgb_camera.exposure'] == 39
    assert params['depth_module.gain'] == params['rgb_camera.gain'] == 64
    assert params['depth_module.depth_profile'] == params['rgb_camera.color_profile'] == params['depth_module.infra_profile'] == '640,480,15'
    assert params['enable_infra1'] is params['enable_sync'] is True
    assert params['depth_module.hdr_enabled'] is False


def test_standalone_d435_default_also_uses_uvc_ticks():
    path = Path(__file__).resolve().parents[1] / 'config/d435-humble.yaml'
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    assert params['rgb_camera.exposure'] == 39
    assert params['rgb_camera.enable_auto_exposure'] is False
    assert params['depth_module.enable_auto_exposure'] is False
    assert params['depth_module.exposure'] == 3900
