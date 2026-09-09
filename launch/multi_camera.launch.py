"""Start declared camera devices as part of the managed robot lifecycle."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.profiles import LAUNCH_FILES
from humanoid_manager.plugin_metadata import resolved_document
from humanoid_manager.plugin_startup import startup_actions


def _launch(context):
    path = Path(LaunchConfiguration('camera_config').perform(context)).expanduser().resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise RuntimeError('camera_config requires schema_version: 1')
    cameras = validate_cameras(document.get('cameras'))
    steps, providers = [], set()
    for camera in cameras:
        if not camera['enabled']:
            continue
        if camera.get('startup'):
            steps.extend((step, {}) for step in resolved_document(camera['startup'], camera))
        elif camera['backend'] in LAUNCH_FILES:
            providers.add(camera['backend'])
    actions = []
    for backend in sorted(providers):
        package, filename = LAUNCH_FILES[backend]
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / filename)),
            launch_arguments={'camera_config': str(path)}.items()))
    return startup_actions(steps, actions)


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_config'), OpaqueFunction(function=_launch)])
