#!/usr/bin/env bash
# Run the logger in the foreground (Ctrl+C stops it cleanly).
# Extra arguments go to the launch file, e.g. bash run.sh ground_host:=192.168.1.20
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$(dirname "$HERE")/ros2_ws/install/setup.bash"
exec ros2 launch uwb_px4_logger logger.launch.py "$@"
