#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for scenario runner moc."""

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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit

from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, OpaqueFunction, TimerAction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario", default="")
    scenario_node = Node(
        package="cpp_mock_scenarios",
        executable=scenario,
        name=scenario,
        output="screen",
        arguments=[("__log_level:=info")],
    )
    shutdown_handler = OnProcessExit(
        target_action=scenario_node,
        on_exit=[
            LogInfo(msg="Shutting down by failure"),
            EmitEvent(event=Shutdown()),
            OpaqueFunction(function=lambda print: sys.exit(-1))
        ]
    )
    timer_action = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Shutting down by success"),
            EmitEvent(event=Shutdown()),
            OpaqueFunction(function=lambda print: sys.exit(0))
        ])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario", default_value=scenario, description="name of the scenario."
            ),
            scenario_node,
            RegisterEventHandler(event_handler=shutdown_handler),
            Node(
                package="simple_sensor_simulator",
                executable="simple_sensor_simulator_node",
                name="simple_sensor_simulator_node",
                output="log",
                arguments=[("__log_level:=warn")],
            ),
            Node(
                package="openscenario_visualization",
                executable="openscenario_visualization_node",
                name="openscenario_visualization_node",
                output="screen",
            ),
            timer_action
        ]
    )
