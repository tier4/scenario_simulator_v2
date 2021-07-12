#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for spline visualization."""

# Copyright (c) 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory(
            "scenario_test_runner"), "config/scenario_simulator_v2.rviz"
    )
    print(rviz_config_dir)
    rosbag_dir = LaunchConfiguration("rosbag_directory", default=Path("/tmp"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rosbag_directory", default_value=rosbag_dir, description="the directory which target rosbag exists."),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                output="screen",
            ),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', rosbag_dir],
                output='screen'
            )
        ]
    )
