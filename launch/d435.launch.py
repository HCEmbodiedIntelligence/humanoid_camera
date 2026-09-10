"""Launch one D435 using the official driver and the timestamp adapter."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from humanoid_camera.depth_processing import DEPTH_FILTER_PARAMETERS
from humanoid_camera.driver_qos import driver_qos_parameters
from humanoid_camera.driver_streams import INFRARED_DEFAULTS


def generate_launch_description():
    default = os.path.join(get_package_share_directory('humanoid_camera'), 'config', 'd435-humble.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('namespace', default_value='front'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('normalize_timestamps', default_value='true'),
        Node(package='humanoid_camera', executable='timestamp_adapter.py',
             name='timestamp_adapter', namespace=LaunchConfiguration('namespace'),
             parameters=[{'source_id': LaunchConfiguration('namespace'),
                          'config_file': LaunchConfiguration('params_file'),
                          'driver_prefix': PathJoinSubstitution(['/', LaunchConfiguration('namespace'), LaunchConfiguration('camera_name')])}],
             condition=IfCondition(LaunchConfiguration('normalize_timestamps')), output='screen'),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name=LaunchConfiguration('camera_name'), namespace=LaunchConfiguration('namespace'),
             parameters=[INFRARED_DEFAULTS, driver_qos_parameters(), LaunchConfiguration('params_file'), {
                 **DEPTH_FILTER_PARAMETERS,
                 'depth_module.enable_auto_exposure': False,
                 'depth_module.exposure': 3900,
                 'rgb_camera.enable_auto_exposure': False,
                 'rgb_camera.exposure': 39,
                 'enable_sync': True,
                 'camera_name': ParameterValue(LaunchConfiguration('camera_name'), value_type=str),
                 'serial_no': ParameterValue(LaunchConfiguration('serial_no'), value_type=str)}], output='screen'),
    ])
