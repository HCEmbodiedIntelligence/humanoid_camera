"""Optional built-in providers. New devices can declare startup directly, without registration."""
from .realsense_profile import validate_cameras
VALIDATORS = {'realsense': validate_cameras}
LAUNCH_FILES = {'realsense': ('humanoid_camera', 'realsense_camera.launch.py')}
