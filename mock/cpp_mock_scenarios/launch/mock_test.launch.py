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

from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, TimerAction, OpaqueFunction
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

def architecture_types():
    return ["awf/universe", "awf/universe/20230906"]


def default_autoware_launch_package_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe": "autoware_launch",
        "awf/universe/20230906": "autoware_launch",
    }[architecture_type]


def default_autoware_launch_file_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe": "planning_simulator.launch.xml",
        "awf/universe/20230906": "planning_simulator.launch.xml",
    }[architecture_type]

def default_rviz_config_file():
    return Path(get_package_share_directory("traffic_simulator")) / "config/scenario_simulator_v2.rviz"

def launch_setup(context, *args, **kwargs):
    # fmt: off
    architecture_type                   = LaunchConfiguration("architecture_type",                      default="awf/universe/20230906")
    autoware_launch_file                = LaunchConfiguration("autoware_launch_file",                   default=default_autoware_launch_file_of(architecture_type.perform(context)))
    autoware_launch_package             = LaunchConfiguration("autoware_launch_package",                default=default_autoware_launch_package_of(architecture_type.perform(context)))
    consider_acceleration_by_road_slope = LaunchConfiguration("consider_acceleration_by_road_slope",    default=False)
    consider_pose_by_road_slope         = LaunchConfiguration("consider_pose_by_road_slope",            default=True)
    global_frame_rate                   = LaunchConfiguration("global_frame_rate",                      default=20.0)
    global_real_time_factor             = LaunchConfiguration("global_real_time_factor",                default=1.0)
    global_timeout                      = LaunchConfiguration("global_timeout",                         default=180)
    initialize_duration                 = LaunchConfiguration("initialize_duration",                    default=300)
    launch_autoware                     = LaunchConfiguration("launch_autoware",                        default=True)
    launch_rviz                         = LaunchConfiguration("launch_rviz",                            default=False)
    launch_simple_sensor_simulator      = LaunchConfiguration("launch_simple_sensor_simulator",         default=True)
    output_directory                    = LaunchConfiguration("output_directory",                       default=Path("/tmp"))
    port                                = LaunchConfiguration("port",                                   default=5555)
    publish_empty_context               = LaunchConfiguration("publish_empty_context",                  default=False)
    record                              = LaunchConfiguration("record",                                 default=False)
    rviz_config                         = LaunchConfiguration("rviz_config",                            default=default_rviz_config_file())
    scenario                            = LaunchConfiguration("scenario",                               default=Path("/dev/null"))
    sensor_model                        = LaunchConfiguration("sensor_model",                           default="")
    sigterm_timeout                     = LaunchConfiguration("sigterm_timeout",                        default=8)
    use_sim_time                        = LaunchConfiguration("use_sim_time",                           default=False)
    vehicle_model                       = LaunchConfiguration("vehicle_model",                          default="")
    scenario_package                    = LaunchConfiguration("package",                                default="cpp_mock_scenarios")
    junit_path                          = LaunchConfiguration("junit_path",                             default="/tmp/output.xunit.xml")
    # fmt: on

    print(f"architecture_type                   := {architecture_type.perform(context)}")
    print(f"autoware_launch_file                := {autoware_launch_file.perform(context)}")
    print(f"autoware_launch_package             := {autoware_launch_package.perform(context)}")
    print(f"consider_acceleration_by_road_slope := {consider_acceleration_by_road_slope.perform(context)}")
    print(f"consider_pose_by_road_slope         := {consider_pose_by_road_slope.perform(context)}")
    print(f"global_frame_rate                   := {global_frame_rate.perform(context)}")
    print(f"global_real_time_factor             := {global_real_time_factor.perform(context)}")
    print(f"global_timeout                      := {global_timeout.perform(context)}")
    print(f"initialize_duration                 := {initialize_duration.perform(context)}")
    print(f"launch_autoware                     := {launch_autoware.perform(context)}")
    print(f"launch_rviz                         := {launch_rviz.perform(context)}")
    print(f"output_directory                    := {output_directory.perform(context)}")
    print(f"port                                := {port.perform(context)}")
    print(f"publish_empty_context               := {publish_empty_context.perform(context)}")
    print(f"record                              := {record.perform(context)}")
    print(f"rviz_config                         := {rviz_config.perform(context)}")
    print(f"scenario                            := {scenario.perform(context)}")
    print(f"sensor_model                        := {sensor_model.perform(context)}")
    print(f"sigterm_timeout                     := {sigterm_timeout.perform(context)}")
    print(f"use_sim_time                        := {use_sim_time.perform(context)}")
    print(f"vehicle_model                       := {vehicle_model.perform(context)}")
    print(f"scenario_package                    := {scenario_package.perform(context)}")
    print(f"junit_path                          := {junit_path.perform(context)}")

    def make_parameters():
        parameters = [
            {"architecture_type": architecture_type},
            {"autoware_launch_file": autoware_launch_file},
            {"autoware_launch_package": autoware_launch_package},
            {"consider_acceleration_by_road_slope": consider_acceleration_by_road_slope},
            {"consider_pose_by_road_slope": consider_pose_by_road_slope},
            {"initialize_duration": initialize_duration},
            {"launch_autoware": launch_autoware},
            {"port": port},
            {"publish_empty_context" : publish_empty_context},
            {"record": record},
            {"rviz_config": rviz_config},
            {"sensor_model": sensor_model},
            {"sigterm_timeout": sigterm_timeout},
            {"vehicle_model": vehicle_model},
            {"global_real_time_factor": global_real_time_factor},
            {"global_frame_rate": global_frame_rate},
            {"global_timeout": global_timeout},
            {"junit_path": junit_path},
        ]
        parameters += make_vehicle_parameters()
        return parameters

    def make_vehicle_parameters():
        parameters = []

        def description():
            return get_package_share_directory(
                vehicle_model.perform(context) + "_description"
            )

        if vehicle_model.perform(context):
            parameters.append(description() + "/config/vehicle_info.param.yaml")
            parameters.append(description() + "/config/simulator_model.param.yaml")
        return parameters

    cpp_scenario_node = Node(
            package=scenario_package,
            executable=scenario,
            name="scenario_node",
            output="screen",
            arguments=[("__log_level:=info")],
            parameters=make_parameters() + [{"use_sim_time": use_sim_time}],
            )
    io_handler = OnProcessIO(
        target_action=cpp_scenario_node,
        on_stderr=on_stderr_output,
        on_stdout=on_stdout_output,
    )
    shutdown_handler = OnProcessExit(
        target_action=cpp_scenario_node, on_exit=[EmitEvent(event=Shutdown())]
    )

    return [
        # fmt: off
        DeclareLaunchArgument("architecture_type",                   default_value=architecture_type                  ),
        DeclareLaunchArgument("autoware_launch_file",                default_value=autoware_launch_file               ),
        DeclareLaunchArgument("autoware_launch_package",             default_value=autoware_launch_package            ),
        DeclareLaunchArgument("consider_acceleration_by_road_slope", default_value=consider_acceleration_by_road_slope),
        DeclareLaunchArgument("consider_pose_by_road_slope",         default_value=consider_pose_by_road_slope        ),
        DeclareLaunchArgument("global_frame_rate",                   default_value=global_frame_rate                  ),
        DeclareLaunchArgument("global_real_time_factor",             default_value=global_real_time_factor            ),
        DeclareLaunchArgument("global_timeout",                      default_value=global_timeout                     ),
        DeclareLaunchArgument("launch_autoware",                     default_value=launch_autoware                    ),
        DeclareLaunchArgument("launch_rviz",                         default_value=launch_rviz                        ),
        DeclareLaunchArgument("publish_empty_context",               default_value=publish_empty_context              ),
        DeclareLaunchArgument("output_directory",                    default_value=output_directory                   ),
        DeclareLaunchArgument("rviz_config",                         default_value=rviz_config                        ),
        DeclareLaunchArgument("scenario",                            default_value=scenario                           ),
        DeclareLaunchArgument("sensor_model",                        default_value=sensor_model                       ),
        DeclareLaunchArgument("sigterm_timeout",                     default_value=sigterm_timeout                    ),
        DeclareLaunchArgument("use_sim_time",                        default_value=use_sim_time                       ),
        DeclareLaunchArgument("vehicle_model",                       default_value=vehicle_model                      ),
        DeclareLaunchArgument("scenario_package",                    default_value=scenario_package                   ),
        DeclareLaunchArgument("junit_path",                          default_value=junit_path                         ),
        # fmt: on
        cpp_scenario_node,
        Node(
            package="simple_sensor_simulator",
            executable="simple_sensor_simulator_node",
            namespace="simulation",
            output="screen",
            parameters=make_parameters() + [{"use_sim_time": use_sim_time}],
            condition=IfCondition(launch_simple_sensor_simulator),
        ),
        Node(
            package="traffic_simulator",
            executable="visualization_node",
            namespace="simulation",
            name="visualizer",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output={"stderr": "log", "stdout": "log"},
            condition=IfCondition(launch_rviz),
            arguments=["-d", str(default_rviz_config_file())],
        ),
        RegisterEventHandler(event_handler=io_handler),
        RegisterEventHandler(event_handler=shutdown_handler),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
