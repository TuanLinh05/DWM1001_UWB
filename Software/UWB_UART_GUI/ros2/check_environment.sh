#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
env | grep '^ROS_'
ros2 pkg executables plotjuggler
ros2 run plotjuggler plotjuggler --version
python3 -c 'import rclpy, serial; print("rclpy and pyserial: OK")'
python3 "$(dirname "$0")/uwb_ros_bridge.py" --help
ros2 bag info /mnt/c/Users/vutua/Downloads/uwb_20261001_200355/uwb_20261001_200355
