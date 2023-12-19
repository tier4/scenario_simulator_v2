#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.conditions import IfCondition

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode

from pathlib import Path

from scenario_test_runner.shutdown_once import ShutdownOnce


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


def launch_setup(context, *args, **kwargs):
    # fmt: off
    architecture_type               = LaunchConfiguration("architecture_type",              default="awf/universe")
    autoware_launch_file            = LaunchConfiguration("autoware_launch_file",           default=default_autoware_launch_file_of(architecture_type.perform(context)))
    autoware_launch_package         = LaunchConfiguration("autoware_launch_package",        default=default_autoware_launch_package_of(architecture_type.perform(context)))
    global_frame_rate               = LaunchConfiguration("global_frame_rate",              default=30.0)
    global_real_time_factor         = LaunchConfiguration("global_real_time_factor",        default=1.0)
    global_timeout                  = LaunchConfiguration("global_timeout",                 default=180)
    initialize_duration             = LaunchConfiguration("initialize_duration",            default=30)
    launch_autoware                 = LaunchConfiguration("launch_autoware",                default=True)
    launch_rviz                     = LaunchConfiguration("launch_rviz",                    default=False)
    launch_simple_sensor_simulator  = LaunchConfiguration("launch_simple_sensor_simulator", default=True)
    output_directory                = LaunchConfiguration("output_directory",               default=Path("/tmp"))
    port                            = LaunchConfiguration("port",                           default=5555)
    record                          = LaunchConfiguration("record",                         default=True)
    rviz_config                     = LaunchConfiguration("rviz_config",                    default="")
    scenario                        = LaunchConfiguration("scenario",                       default=Path("/dev/null"))
    sensor_model                    = LaunchConfiguration("sensor_model",                   default="")
    sigterm_timeout                 = LaunchConfiguration("sigterm_timeout",                default=8)
    vehicle_model                   = LaunchConfiguration("vehicle_model",                  default="")
    workflow                        = LaunchConfiguration("workflow",                       default=Path("/dev/null"))
    # fmt: on

    print(f"architecture_type       := {architecture_type.perform(context)}")
    print(f"autoware_launch_file    := {autoware_launch_file.perform(context)}")
    print(f"autoware_launch_package := {autoware_launch_package.perform(context)}")
    print(f"global_frame_rate       := {global_frame_rate.perform(context)}")
    print(f"global_real_time_factor := {global_real_time_factor.perform(context)}")
    print(f"global_timeout          := {global_timeout.perform(context)}")
    print(f"initialize_duration     := {initialize_duration.perform(context)}")
    print(f"launch_autoware         := {launch_autoware.perform(context)}")
    print(f"launch_rviz             := {launch_rviz.perform(context)}")
    print(f"output_directory        := {output_directory.perform(context)}")
    print(f"port                    := {port.perform(context)}")
    print(f"record                  := {record.perform(context)}")
    print(f"rviz_config             := {rviz_config.perform(context)}")
    print(f"scenario                := {scenario.perform(context)}")
    print(f"sensor_model            := {sensor_model.perform(context)}")
    print(f"sigterm_timeout         := {sigterm_timeout.perform(context)}")
    print(f"vehicle_model           := {vehicle_model.perform(context)}")
    print(f"workflow                := {workflow.perform(context)}")

    def make_parameters():
        parameters = [
            {"architecture_type": architecture_type},
            {"autoware_launch_file": autoware_launch_file},
            {"autoware_launch_package": autoware_launch_package},
            {"initialize_duration": initialize_duration},
            {"launch_autoware": launch_autoware},
            {"port": port},
            {"record": record},
            {"rviz_config": rviz_config},
            {"sensor_model": sensor_model},
            {"vehicle_model": vehicle_model},
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

    return [
        # fmt: off
        DeclareLaunchArgument("architecture_type",       default_value=architecture_type      ),
        DeclareLaunchArgument("autoware_launch_file",    default_value=autoware_launch_file   ),
        DeclareLaunchArgument("autoware_launch_package", default_value=autoware_launch_package),
        DeclareLaunchArgument("global_frame_rate",       default_value=global_frame_rate      ),
        DeclareLaunchArgument("global_real_time_factor", default_value=global_real_time_factor),
        DeclareLaunchArgument("global_timeout",          default_value=global_timeout         ),
        DeclareLaunchArgument("launch_autoware",         default_value=launch_autoware        ),
        DeclareLaunchArgument("launch_rviz",             default_value=launch_rviz            ),
        DeclareLaunchArgument("output_directory",        default_value=output_directory       ),
        DeclareLaunchArgument("rviz_config",             default_value=rviz_config            ),
        DeclareLaunchArgument("scenario",                default_value=scenario               ),
        DeclareLaunchArgument("sensor_model",            default_value=sensor_model           ),
        DeclareLaunchArgument("sigterm_timeout",         default_value=sigterm_timeout        ),
        DeclareLaunchArgument("vehicle_model",           default_value=vehicle_model          ),
        DeclareLaunchArgument("workflow",                default_value=workflow               ),
        # fmt: on
        Node(
            package="scenario_test_runner",
            executable="scenario_test_runner.py",
            namespace="simulation",
            name="scenario_test_runner",
            output="screen",
            on_exit=ShutdownOnce(),
            arguments=[
                # fmt: off
                "--global-frame-rate",       global_frame_rate,
                "--global-real-time-factor", global_real_time_factor,
                "--global-timeout",          global_timeout,
                "--output-directory",        output_directory,
                "--scenario",                scenario,
                "--workflow",                workflow,
                # fmt: on
            ],
        ),
        Node(
            package="simple_sensor_simulator",
            executable="simple_sensor_simulator_node",
            namespace="simulation",
            output="screen",
            on_exit=ShutdownOnce(),
            parameters=[{"port": port}, {"use_sim_time": True}]+make_vehicle_parameters(),
            condition=IfCondition(launch_simple_sensor_simulator),
        ),
        # The `name` keyword overrides the name for all created nodes, so duplicated nodes appear.
        # For LifecycleNode the `name` parameter is required
        # For Node the `name` parameter is optional
        # The LifecycleNode class inherits from Node class with some additions not used in this launch file
        # In this case, `openscenario_interpreter_node` can be changed from Lifecycle to Node
        Node(
            package="openscenario_interpreter",
            executable="openscenario_interpreter_node",
            namespace="simulation",
            output="screen",
            parameters=[{"use_sim_time": True}]+make_parameters(),
            on_exit=ShutdownOnce(),
        ),
        Node(
            package="openscenario_preprocessor",
            executable="openscenario_preprocessor_node",
            namespace="simulation",
            output="screen",
            on_exit=ShutdownOnce(),
        ),
        Node(
            package="openscenario_visualization",
            executable="openscenario_visualization_node",
            namespace="simulation",
            name="openscenario_visualizer",
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
                    Path(get_package_share_directory("traffic_simulator"))
                    / "config/scenario_simulator_v2.rviz"
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
