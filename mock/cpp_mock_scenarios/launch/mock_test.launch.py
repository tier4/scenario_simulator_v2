#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for scenario runner moc."""

# Copyright (c) 2020 TIER IV, Inc. All rights reserved.
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

from logging import shutdown
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch

from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnProcessIO

from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, TimerAction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node

from typing import cast

from pathlib import Path


class Color:
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    PURPLE = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    END = "\033[0m"
    BOLD = "\038[1m"
    UNDERLINE = "\033[4m"
    INVISIBLE = "\033[08m"
    REVERSE = "\033[07m"


def on_stderr_output(event: launch.Event) -> None:
    lines = event.text.decode().splitlines()
    if len(lines) == 1:
        if lines[0] == "cpp_scenario:failure":
            print(Color.RED + "Scenario Failed" + Color.END)


def on_stdout_output(event: launch.Event) -> None:
    lines = event.text.decode().splitlines()
    if len(lines) == 1:
        if lines[0] == "cpp_scenario:success":
            print(Color.GREEN + "Scenario Succeed" + Color.END)


def generate_launch_description():
    timeout = LaunchConfiguration("timeout", default=10.0)
    scenario = LaunchConfiguration("scenario", default="")
    scenario_package = LaunchConfiguration("package", default="cpp_mock_scenarios")
    junit_path = LaunchConfiguration("junit_path", default="/tmp/output.xunit.xml")
    launch_rviz = LaunchConfiguration("launch_rviz", default=False)
    vehicle_model = LaunchConfiguration("vehicle_model", default="lexus")
    sensor_model = LaunchConfiguration("sensor_model", default="aip_xx1")
    scenario_node = Node(
        package=scenario_package,
        executable=scenario,
        name="scenario_node",
        output="screen",
        arguments=[("__log_level:=info")],
        parameters=[
            {
                "junit_path": junit_path,
                "timeout": timeout,
                "architecture_type": "awf/universe",
                "autoware_launch_file": "planning_simulator.launch.xml",
                "autoware_launch_package": "autoware_launch",
                "launch_autoware": True,
                "vehicle_model": vehicle_model,
                "sensor_model": sensor_model,
                "initialize_duration": 900
            }
        ],
    )
    io_handler = OnProcessIO(
        target_action=scenario_node,
        on_stderr=on_stderr_output,
        on_stdout=on_stdout_output,
    )
    shutdown_handler = OnProcessExit(
        target_action=scenario_node, on_exit=[EmitEvent(event=Shutdown())]
    )
    description = LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario", default_value=scenario, description="Name of the scenario."
            ),
            DeclareLaunchArgument(
                "package",
                default_value=scenario_package,
                description="Name of package your scenario exists",
            ),
            DeclareLaunchArgument(
                "timeout", default_value=timeout, description="Timeout in seconds."
            ),
            DeclareLaunchArgument(
                "junit_path",
                default_value=junit_path,
                description="Path of the junit output.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value=launch_rviz,
                description="If true, launch with rviz.",
            ),
            DeclareLaunchArgument(
                "vehicle_model",
                default_value=vehicle_model,
                description="Vehicle model of the Autoware",
            ),
            DeclareLaunchArgument(
                "sensor_model",
                default_value=sensor_model,
                description="Sensor model of the Autoware",
            ),
            scenario_node,
            RegisterEventHandler(event_handler=io_handler),
            RegisterEventHandler(event_handler=shutdown_handler),
            Node(
                package="simple_sensor_simulator",
                executable="simple_sensor_simulator_node",
                name="simple_sensor_simulator_node",
                output="log",
                arguments=[("__log_level:=warn")],
            ),
            Node(
                package="traffic_simulator",
                executable="visualization_node",
                name="visualizer",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output={"stderr": "log", "stdout": "log"},
                condition=IfCondition(launch_rviz),
                arguments=[
                    "-d",
                    str(
                        Path(get_package_share_directory("cpp_mock_scenarios"))
                        / "rviz/mock_test.rviz"
                    ),
                ],
            ),
        ]
    )
    return description
