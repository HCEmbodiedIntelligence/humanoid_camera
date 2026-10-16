#!/usr/bin/env bash
set -euo pipefail
camera_package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [ -f "$camera_package_dir/../../setup_hc.bash" ]; then
  source "$camera_package_dir/../../setup_hc.bash"
elif [ -f "$camera_package_dir/../../install/setup.bash" ]; then
  source "$camera_package_dir/../../install/setup.bash"
elif [ -f "$camera_package_dir/../../../setup.bash" ]; then
  source "$camera_package_dir/../../../setup.bash"
fi
if [[ $# == 0 || "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: start_cameras.sh /path/to/robot-profile/cameras.yaml [launch arguments]'
  echo 'For HC use: bash tools/run_cameras.sh --profile adapters/robots/openarmx'
  [[ $# != 0 ]] && exit 0
  exit 2
fi
config=$1
[[ -f "$config" ]] || { echo "Camera config not found: $config" >&2; exit 2; }
shift
exec ros2 launch humanoid_camera multi_camera.launch.py camera_config:="$config" "$@"
