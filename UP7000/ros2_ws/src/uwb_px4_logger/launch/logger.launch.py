"""Start uwb_relay, px4_logger and up4000_status sharing one session folder.

ros2 launch uwb_px4_logger logger.launch.py [ground_host:=192.168.1.20]
    [log_root:=~/flight_logs] [session:=test01] [config:=/path/logger.yaml]
"""
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    # value_type=str: launch would otherwise YAML-parse "20261107_150000" as an int.
    text = lambda value: ParameterValue(value, value_type=str)  # noqa: E731
    session = get("session") or datetime.now().strftime("%Y%m%d_%H%M%S")
    shared = {"log_root": text(get("log_root")), "session": text(session)}
    if get("ground_host"):
        shared["ground_host"] = text(get("ground_host"))  # empty keeps the YAML/auto value
    config = get("config")
    return [
        Node(package="uwb_px4_logger", executable="uwb_relay", name="uwb_relay",
             parameters=[config, shared], output="screen"),
        Node(package="uwb_px4_logger", executable="px4_logger", name="px4_logger",
             parameters=[config, shared], output="screen"),
        Node(package="uwb_px4_logger", executable="up4000_status", name="up4000_status",
             parameters=[config, shared], output="screen"),
    ]


def generate_launch_description():
    share = get_package_share_directory("uwb_px4_logger")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "logger.yaml")),
        DeclareLaunchArgument("log_root", default_value="~/flight_logs"),
        DeclareLaunchArgument("session", default_value=""),
        DeclareLaunchArgument("ground_host", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
