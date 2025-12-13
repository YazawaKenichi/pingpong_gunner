#!/usr/bin/env python3
# coding : utf-8
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial = DeclareLaunchArgument(
            "serial",
            default_value="/dev/ttyACM0",
            description="Serial device path for micro-ROS agent (e.g. /dev/ttyACM0)"
            )

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_serial",
        output="screen",
        arguments=[
            "serial",
            "-b", "115200",
            "--dev", LaunchConfiguration("serial"),
            "-v6"
        ]
    )

    return LaunchDescription([
        serial,
        micro_ros_agent_node,
    ])


