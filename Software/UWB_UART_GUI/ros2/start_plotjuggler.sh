#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
exec ros2 run plotjuggler plotjuggler "$@"
