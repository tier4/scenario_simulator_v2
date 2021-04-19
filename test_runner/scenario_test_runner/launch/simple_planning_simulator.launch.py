#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Tier IV, Inc. All rights reserved.
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

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    topic_tools = Node(
        package="topic_tools",
        executable='relay',
        parameters=[{
            'use_sim_time': EnvironmentVariable(name="AW_ROS2_USE_SIM_TIME", default_value="False"),
            'input_topic': '/vehicle/status/twist',
            'output_topic': '/localization/twist',
            'type': 'geometry_msgs/msg/TwistStamped'
        }]
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('output_directory')],
        output='log',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30),
        sigkill_timeout=LaunchConfiguration('sigkill_timeout', default=30),
    )

    return LaunchDescription([
        topic_tools,
        rosbag_record,
    ])