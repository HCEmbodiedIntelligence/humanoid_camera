#!/usr/bin/env bash
set -e
camera_package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
if [ -f "$camera_package_dir/../../install/setup.bash" ]; then
  source "$camera_package_dir/../../install/setup.bash"
elif [ -f "$camera_package_dir/../../../setup.bash" ]; then
  source "$camera_package_dir/../../../setup.bash"
fi
camera_overlay="$HOME/.local/share/humanoid-camera/ros-overlay/opt/ros/humble"
if [ -f "$camera_overlay/share/realsense2_camera/package.xml" ]; then
  export AMENT_PREFIX_PATH="$camera_overlay:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$camera_overlay/lib:$camera_overlay/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
exec ros2 launch humanoid_camera d405.launch.py "$@"
