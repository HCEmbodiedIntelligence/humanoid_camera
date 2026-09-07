#!/usr/bin/env bash
set -euo pipefail
camera_package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
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
default_config="$(ros2 pkg prefix --share humanoid_camera)/config/three-camera.example.yaml"
config=${1:-"$default_config"}
shift || true
exec ros2 launch humanoid_camera multi_camera.launch.py camera_config:="$config" "$@"
