#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 "$task_dir/uwb_ros_bridge.py" "$@"
