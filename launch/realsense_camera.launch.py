"""RealSense provider: official driver plus timestamp adapter."""
from pathlib import Path
import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from humanoid_manager.plugin_metadata import resolved_document
from humanoid_camera.exposure import D435_RGB_MODELS, rgb_exposure_parameter


def _parameters(camera):
    width, height, fps = (int(camera[key]) for key in ('width', 'height', 'fps'))
    device = str(camera['device_type']).lower()
    depth_auto = bool(camera.get('depth_auto_exposure', True))
    params = {
        'device_type': str(camera['device_type']),
        'serial_no': str(camera['serial_no']),
        'camera_name': str(camera['camera_name']),
        'enable_color': True,
        'enable_depth': True,
        'depth_module.depth_profile': f'{width},{height},{fps}',
        'depth_module.depth_format': str(camera.get('depth_format', 'Z16')),
        'depth_module.enable_auto_exposure': depth_auto,
        'depth_module.global_time_enabled': True,
        'enable_sync': bool(camera.get('sync_rgb_depth', True)),
        'pointcloud.enable': bool(camera.get('pointcloud', False)),
        'pointcloud.stream_filter': 2,
        'pointcloud.stream_index_filter': 0,
        'align_depth.enable': bool(camera.get('align_depth', False)),
        'temporal_filter.enable': False,
        'hdr_merge.enable': False,
        'depth_module.hdr_enabled': False,
        'publish_tf': True,
        'tf_prefix': str(camera['id']) + '_',
    }
    if depth_auto:
        params.update({
            'depth_module.auto_exposure_limit': int(camera.get('depth_auto_exposure_limit_us', 4500)),
            'depth_module.auto_gain_limit': int(camera.get('depth_auto_gain_limit', 64)),
            'depth_module.auto_exposure_limit_toggle': True,
            'depth_module.auto_gain_limit_toggle': True,
        })
    else:
        params.update({
            'depth_module.exposure': int(camera.get('depth_exposure_us', 4500)),
            'depth_module.gain': int(camera.get('depth_gain', 64)),
        })
    if device == 'd405':
        params.update({
            'depth_module.color_profile': f'{width},{height},{fps}',
            'depth_module.color_format': str(camera.get('color_format', 'RGB8')),
        })
    else:
        color_auto = bool(camera.get('color_auto_exposure', False))
        params.update({
            'rgb_camera.color_profile': f'{width},{height},{fps}',
            'rgb_camera.color_format': str(camera.get('color_format', 'RGB8')),
            'rgb_camera.enable_auto_exposure': color_auto,
            'rgb_camera.global_time_enabled': True,
        })
        if not color_auto:
            params.update({
                'rgb_camera.exposure': rgb_exposure_parameter(device, camera.get('color_exposure_us', 4500)),
                'rgb_camera.gain': int(camera.get('color_gain', 64)),
            })
    params.update(camera.get('parameters', {}))
    # Identity and stream selection cannot be replaced by advanced parameters.
    params.update(device_type=str(camera['device_type']), serial_no=str(camera['serial_no']),
                  camera_name=str(camera['camera_name']), enable_color=True, enable_depth=True)
    if device in D435_RGB_MODELS:
        params.update({
            'rgb_camera.enable_auto_exposure': False,
            'rgb_camera.exposure': rgb_exposure_parameter(device, camera.get('color_exposure_us', 4500)),
            'rgb_camera.gain': int(camera.get('color_gain', 64)),
        })
    return params


def _launch(context):
    config_path = Path(LaunchConfiguration('camera_config').perform(context)).expanduser().resolve()
    document = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1 or not isinstance(document.get('cameras'), list):
        raise RuntimeError('camera_config must contain schema_version: 1 and cameras: []')
    from humanoid_camera.configuration import validate_cameras
    document['cameras'] = validate_cameras(document['cameras'])
    enabled = [c for c in document['cameras'] if not c.get('startup') and c.get('enabled', True) and c.get('backend', 'realsense') == 'realsense']
    if len(enabled) > 1 and any(not str(c.get('serial_no', '')) for c in enabled):
        raise RuntimeError('every enabled RealSense needs a serial_no when launching multiple cameras')
    actions = []
    for camera in enabled:
        camera = resolved_document(camera, camera)
        ident, namespace, name = (str(camera.get(key, '')) for key in ('id', 'namespace', 'camera_name'))
        if not all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,63}', value) for value in (ident, namespace, name)):
            raise RuntimeError(f'invalid camera ROS name: {ident}/{namespace}/{name}')
        prefix = f'/{namespace}/{name}'
        # ROS launch otherwise parses digit-only serial numbers as integers.
        parameters = {key: ParameterValue(value, value_type=str) if isinstance(value, str) else value
                      for key, value in _parameters(camera).items()}
        image_remappings = [(prefix + '/' + suffix, camera[key]) for key, suffix in (
            ('rgb_topic', 'color/image_raw'), ('depth_topic', 'depth/image_rect_raw'))]
        actions.append(Node(package='realsense2_camera', executable='realsense2_camera_node',
                            name=name, namespace=namespace, parameters=[parameters], remappings=image_remappings,
                            output='screen', on_exit=Shutdown(reason=f'camera {ident} exited')))
        actions.append(Node(package='humanoid_camera', executable='timestamp_adapter.py',
                            name=name + '_timestamp_adapter', namespace=namespace,
                            parameters=[{'source_id': ParameterValue(ident, value_type=str),
                                         'config_file': ParameterValue(str(config_path), value_type=str),
                                         'driver_prefix': ParameterValue(prefix, value_type=str)}],
                            remappings=image_remappings + [('normalized/' + suffix, camera[key]) for key, suffix in (
                                ('rgbd_topic', 'rgbd'), ('metadata_topic', 'metadata'),
                                ('pointcloud_topic', 'points'), ('pointcloud_metadata_topic', 'points_metadata'))],
            condition=IfCondition(str(bool(camera.get('timestamp_alignment', camera.get('normalize_timestamps', True)))).lower()),
                            output='screen', on_exit=Shutdown(reason=f'camera {ident} timestamp adapter exited')))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', description='Robot camera YAML generated by humanoid_manager'),
        OpaqueFunction(function=_launch),
    ])
