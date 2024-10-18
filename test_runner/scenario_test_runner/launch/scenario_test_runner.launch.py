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

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode

from pathlib import Path

from scenario_test_runner.shutdown_once import ShutdownOnce


def architecture_types():
    return ["awf/universe/20230906", "awf/universe/20240605"]


def default_autoware_launch_package_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe/20230906": "autoware_launch",
        "awf/universe/20240605": "autoware_launch",
    }[architecture_type]


def default_autoware_launch_file_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe/20230906": "planning_simulator.launch.xml",
        "awf/universe/20240605": "planning_simulator.launch.xml",
    }[architecture_type]


def default_rviz_config_file():
    return Path(get_package_share_directory("traffic_simulator")) / "config/scenario_simulator_v2.rviz"


def launch_setup(context, *args, **kwargs):
    # fmt: off
    architecture_type                   = LaunchConfiguration("architecture_type",                      default="awf/universe/20240605")
    autoware_launch_file                = LaunchConfiguration("autoware_launch_file",                   default=default_autoware_launch_file_of(architecture_type.perform(context)))
    autoware_launch_package             = LaunchConfiguration("autoware_launch_package",                default=default_autoware_launch_package_of(architecture_type.perform(context)))
    consider_acceleration_by_road_slope = LaunchConfiguration("consider_acceleration_by_road_slope",    default=False)
    consider_pose_by_road_slope         = LaunchConfiguration("consider_pose_by_road_slope",            default=True)
    enable_perf                         = LaunchConfiguration("enable_perf",                            default=False)
    global_frame_rate                   = LaunchConfiguration("global_frame_rate",                      default=30.0)
    global_real_time_factor             = LaunchConfiguration("global_real_time_factor",                default=1.0)
    global_timeout                      = LaunchConfiguration("global_timeout",                         default=180)
    initialize_duration                 = LaunchConfiguration("initialize_duration",                    default=30)
    launch_autoware                     = LaunchConfiguration("launch_autoware",                        default=True)
    launch_rviz                         = LaunchConfiguration("launch_rviz",                            default=False)
    launch_simple_sensor_simulator      = LaunchConfiguration("launch_simple_sensor_simulator",         default=True)
    output_directory                    = LaunchConfiguration("output_directory",                       default=Path("/tmp"))
    port                                = LaunchConfiguration("port",                                   default=5555)
    publish_empty_context               = LaunchConfiguration("publish_empty_context",                  default=False)
    record                              = LaunchConfiguration("record",                                 default=True)
    rviz_config                         = LaunchConfiguration("rviz_config",                            default=default_rviz_config_file())
    scenario                            = LaunchConfiguration("scenario",                               default=Path("/dev/null"))
    sensor_model                        = LaunchConfiguration("sensor_model",                           default="")
    sigterm_timeout                     = LaunchConfiguration("sigterm_timeout",                        default=8)
    use_sim_time                        = LaunchConfiguration("use_sim_time",                           default=False)
    vehicle_model                       = LaunchConfiguration("vehicle_model",                          default="")
    # fmt: on

    print(f"architecture_type                   := {architecture_type.perform(context)}")
    print(f"autoware_launch_file                := {autoware_launch_file.perform(context)}")
    print(f"autoware_launch_package             := {autoware_launch_package.perform(context)}")
    print(f"consider_acceleration_by_road_slope := {consider_acceleration_by_road_slope.perform(context)}")
    print(f"consider_pose_by_road_slope         := {consider_pose_by_road_slope.perform(context)}")
    print(f"enable_perf                         := {enable_perf.perform(context)}")
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

    def make_launch_prefix():
        if enable_perf.perform(context) == "True":
            return "perf record -F 10000"
        else:
            return ""

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
            {"use_sim_time": use_sim_time},
            {"vehicle_model": vehicle_model},
        ]

        def collect_vehicle_parameters():
            if vehicle_model_name := vehicle_model.perform(context):
                description = get_package_share_directory(vehicle_model_name + "_description")
                return [
                    description + "/config/vehicle_info.param.yaml",
                    description + "/config/simulator_model.param.yaml",
                ]
            else:
                return []

        if (it := collect_vehicle_parameters()) != []:
            parameters += it

        def collect_prefixed_parameters():
            return [item[0][9:] + ':=' + item[1] for item in context.launch_configurations.items() if item[0][:9] == 'autoware.']

        if (it := collect_prefixed_parameters()) != []:
            parameters += [{"autoware.": it}]

        return parameters

    return [
        # fmt: off
        DeclareLaunchArgument("architecture_type",                   default_value=architecture_type                  ),
        DeclareLaunchArgument("autoware_launch_file",                default_value=autoware_launch_file               ),
        DeclareLaunchArgument("autoware_launch_package",             default_value=autoware_launch_package            ),
        DeclareLaunchArgument("consider_acceleration_by_road_slope", default_value=consider_acceleration_by_road_slope),
        DeclareLaunchArgument("consider_pose_by_road_slope",         default_value=consider_pose_by_road_slope        ),
        DeclareLaunchArgument("enable_perf",                         default_value=enable_perf                        ),
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
                # fmt: on
            ],
        ),
        Node(
            package="simple_sensor_simulator",
            executable="simple_sensor_simulator_node",
            namespace="simulation",
            output="screen",
            on_exit=ShutdownOnce(),
            parameters=make_parameters(),
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
            parameters=make_parameters(),
            prefix=make_launch_prefix(),
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
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
