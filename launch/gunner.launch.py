#!/usr/bin/env python3
# coding : utf-8
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pingpong_gunner"),
        "config",
        "gunner.param.yaml"
    )

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent_serial",
        output="screen",
        arguments=[
            "serial",
            "-b", "115200",
            "--dev", "/dev/ttyACM0",
            "-v6"
        ]
    )

    cpp_pingpong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("cpp_pingpong"),
                "launch",
                "pingpong.launch.py"
            )
        ])
    )

    gunner_node = Node(
        package="pingpong_gunner",
        executable="pingpong_gunner_exe",
        output="screen",
        parameters=[config]
    )

    return LaunchDescription([
        micro_ros_agent_node,
        cpp_pingpong_launch,
        gunner_node,
    ])


