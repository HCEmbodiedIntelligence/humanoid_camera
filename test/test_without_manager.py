import os
import subprocess
import sys


def test_installed_camera_and_startup_without_manager():
    code = """
import sys
class RejectManager:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'humanoid_manager' or fullname.startswith('humanoid_manager.'):
            raise AssertionError('legacy manager import: ' + fullname)
sys.meta_path.insert(0, RejectManager())
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.errors import CameraConfigurationError
from humanoid_camera.settings import resolved_document
from robot_bringup.startup import validate_startup, startup_actions
from robot_bringup.ros2_controllers import ensure_controllers
assert validate_cameras([{'id': 'front', 'backend': 'ros_topics'}])[0]['id'] == 'front'
try:
    validate_cameras([{'id': 'front', 'startup': [{'kind': 'shell'}]}])
except CameraConfigurationError:
    pass
else:
    raise AssertionError('invalid startup accepted')
assert not any(n.startswith('humanoid_manager') for n in sys.modules)
"""
    subprocess.run([sys.executable, '-s', '-c', code], check=True, env=os.environ.copy())
