#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for the rollout test runner."""

# Copyright (c) 2026 TIER IV, Inc. All rights reserved.
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


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


def default_rollout_param_file():
    return Path(get_package_share_directory("rollout_test_runner")) / "config/rollout.param.yaml"


def launch_setup(context, *args, **kwargs):
    # fmt: off
    architecture_type                   = LaunchConfiguration("architecture_type",                default="awf/universe/20240605")
    auto_switch_time                    = LaunchConfiguration("auto_switch_time",                 default=-1.0)
    autoware_launch_file                = LaunchConfiguration("autoware_launch_file",             default=default_autoware_launch_file_of(architecture_type.perform(context)))
    autoware_launch_package             = LaunchConfiguration("autoware_launch_package",          default=default_autoware_launch_package_of(architecture_type.perform(context)))
    bag_path                            = LaunchConfiguration("bag_path")
    consider_acceleration_by_road_slope = LaunchConfiguration("consider_acceleration_by_road_slope", default=False)
    consider_pose_by_road_slope         = LaunchConfiguration("consider_pose_by_road_slope",      default=True)
    global_frame_rate                   = LaunchConfiguration("global_frame_rate",                default=20.0)
    global_real_time_factor             = LaunchConfiguration("global_real_time_factor",          default=1.0)
    global_timeout                      = LaunchConfiguration("global_timeout",                   default=0.0)
    goal_pose                           = LaunchConfiguration("goal_pose",                        default="[]")
    initialize_duration                 = LaunchConfiguration("initialize_duration",              default=300)
    launch_autoware                     = LaunchConfiguration("launch_autoware",                  default=True)
    launch_rviz                         = LaunchConfiguration("launch_rviz",                      default=False)
    launch_simple_sensor_simulator      = LaunchConfiguration("launch_simple_sensor_simulator",   default=True)
    map_path                            = LaunchConfiguration("map_path")
    parameter_file_path                 = LaunchConfiguration("parameter_file_path",              default=Path(get_package_share_directory("scenario_test_runner")) / "config/parameters.yaml")
    port                                = LaunchConfiguration("port",                             default=5555)
    record                              = LaunchConfiguration("record",                           default=False)
    replay_start_offset                 = LaunchConfiguration("replay_start_offset",              default=0.0)
    rollout_param_file                  = LaunchConfiguration("rollout_param_file",               default=default_rollout_param_file())
    rviz_config                         = LaunchConfiguration("rviz_config",                      default=default_rviz_config_file())
    sensor_model                        = LaunchConfiguration("sensor_model",                     default="")
    sigterm_timeout                     = LaunchConfiguration("sigterm_timeout",                  default=8)
    simulate_localization               = LaunchConfiguration("simulate_localization",            default=True)
    vehicle_model                       = LaunchConfiguration("vehicle_model",                    default="")
    ego_model                           = LaunchConfiguration("ego_model",                        default="")
    # fmt: on

    print(f"architecture_type        := {architecture_type.perform(context)}")
    print(f"autoware_launch_file     := {autoware_launch_file.perform(context)}")
    print(f"autoware_launch_package  := {autoware_launch_package.perform(context)}")
    print(f"bag_path                 := {bag_path.perform(context)}")
    print(f"global_frame_rate        := {global_frame_rate.perform(context)}")
    print(f"global_real_time_factor  := {global_real_time_factor.perform(context)}")
    print(f"goal_pose                := {goal_pose.perform(context)}")
    print(f"map_path                 := {map_path.perform(context)}")
    print(f"rollout_param_file       := {rollout_param_file.perform(context)}")
    print(f"sensor_model             := {sensor_model.perform(context)}")
    print(f"vehicle_model            := {vehicle_model.perform(context)}")

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
            {"record": record},
            {"rviz_config": rviz_config},
            {"sensor_model": sensor_model},
            {"sigterm_timeout": sigterm_timeout},
            {"simulate_localization": simulate_localization},
            {"vehicle_model": vehicle_model},
            {"ego_model": ego_model},
            {"global_real_time_factor": global_real_time_factor},
            {"global_frame_rate": global_frame_rate},
            {"global_timeout": global_timeout},
            {"map_path": map_path},
            # All nodes must follow the rosbag-aligned simulation clock.
            {"use_sim_time": True},
        ]
        parameters += make_vehicle_parameters()
        parameters += [parameter_file_path.perform(context)]
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

    def make_rollout_parameters():
        parameters = make_parameters()
        parameters += [rollout_param_file.perform(context)]
        parameters += [
            {"bag_path": bag_path},
            {"replay_start_offset": replay_start_offset},
            {"auto_switch_time": auto_switch_time},
        ]
        if goal_pose.perform(context) not in ("", "[]"):
            parameters += [{"goal_pose": goal_pose}]
        return parameters

    return [
        # fmt: off
        DeclareLaunchArgument("architecture_type",        default_value=architecture_type),
        DeclareLaunchArgument("auto_switch_time",         default_value=auto_switch_time),
        DeclareLaunchArgument("autoware_launch_file",     default_value=autoware_launch_file),
        DeclareLaunchArgument("autoware_launch_package",  default_value=autoware_launch_package),
        DeclareLaunchArgument("bag_path",                 description="Path to the rosbag directory to replay"),
        DeclareLaunchArgument("global_frame_rate",        default_value=global_frame_rate),
        DeclareLaunchArgument("global_real_time_factor",  default_value=global_real_time_factor),
        DeclareLaunchArgument("global_timeout",           default_value=global_timeout),
        DeclareLaunchArgument("goal_pose",                default_value=goal_pose, description="Goal pose [x, y, z, roll, pitch, yaw] in the map frame"),
        DeclareLaunchArgument("launch_autoware",          default_value=launch_autoware),
        DeclareLaunchArgument("launch_rviz",              default_value=launch_rviz),
        DeclareLaunchArgument("map_path",                 description="Directory containing lanelet2 (.osm) and pointcloud (.pcd) maps"),
        DeclareLaunchArgument("parameter_file_path",      default_value=parameter_file_path),
        DeclareLaunchArgument("replay_start_offset",      default_value=replay_start_offset),
        DeclareLaunchArgument("rollout_param_file",       default_value=rollout_param_file),
        DeclareLaunchArgument("rviz_config",              default_value=rviz_config),
        DeclareLaunchArgument("sensor_model",             default_value=sensor_model),
        DeclareLaunchArgument("sigterm_timeout",          default_value=sigterm_timeout),
        DeclareLaunchArgument("simulate_localization",    default_value=simulate_localization),
        DeclareLaunchArgument("vehicle_model",            default_value=vehicle_model),
        DeclareLaunchArgument("ego_model",                default_value=ego_model),
        # fmt: on
        Node(
            package="rollout_test_runner",
            executable="rollout_test_runner_node",
            name="rollout_test_runner",
            output="screen",
            arguments=[("__log_level:=info")],
            parameters=make_rollout_parameters(),
        ),
        Node(
            package="simple_sensor_simulator",
            executable="simple_sensor_simulator_node",
            namespace="simulation",
            output="screen",
            parameters=make_parameters(),
            condition=IfCondition(launch_simple_sensor_simulator),
        ),
        Node(
            package="traffic_simulator",
            executable="visualization_node",
            namespace="simulation",
            name="visualizer",
            output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=[("/simulation/entity/status", "/entity/status")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output={"stderr": "log", "stdout": "log"},
            condition=IfCondition(launch_rviz),
            arguments=["-d", str(default_rviz_config_file())],
            parameters=[{"use_sim_time": True}],
            remappings=[
                ("/simulation/lanelet/marker", "/lanelet/marker"),
                ("/simulation/debug_marker", "/debug_marker"),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
