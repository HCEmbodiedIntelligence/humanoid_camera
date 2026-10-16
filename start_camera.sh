#!/usr/bin/env bash
# Compatibility spelling; device settings come from cameras.yaml.
set -euo pipefail
camera_package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$camera_package_dir/start_cameras.sh" "$@"
