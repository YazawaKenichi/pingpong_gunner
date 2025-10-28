#!/usr/bin/env python3
# coding : utf-8
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory("pingpong_gunner"), "config", "gunner.param.yaml")
    return LaunchDescription([
        Node(package = "pingpong_gunner", executable = "pingpong_gunner_exe", output = "screen", parameters = [config])
        ])

