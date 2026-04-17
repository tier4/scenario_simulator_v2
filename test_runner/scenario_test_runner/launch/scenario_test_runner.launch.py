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

import os
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode

from pathlib import Path

from scenario_test_runner.shutdown_once import ShutdownOnce


def architecture_types():
    # awf/universe/20230906: autoware_perception_msgs/TrafficSignalArray for traffic lights
    # awf/universe/20240605: autoware_perception_msgs/TrafficLightGroupArray for traffic lights
    # awf/universe/20250130: [Pilot.Auto >= 0.41.1] autoware_internal_planning_msgs/msg/PathWithLaneId for concealer
    return ["awf/universe/20230906", "awf/universe/20240605", "awf/universe/20250130"]


def default_autoware_launch_package_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe/20230906": "autoware_launch",
        "awf/universe/20240605": "autoware_launch",
        "awf/universe/20250130": "autoware_launch",
    }[architecture_type]


def default_autoware_launch_file_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type := {architecture_type} is not supported. Choose one of {architecture_types()}."
        )
    return {
        "awf/universe/20230906": "planning_simulator.launch.xml",
        "awf/universe/20240605": "planning_simulator.launch.xml",
        "awf/universe/20250130": "planning_simulator.launch.xml",
    }[architecture_type]


def default_rviz_config_file():
    return Path(get_package_share_directory("traffic_simulator")) / "config/scenario_simulator_v2.rviz"


def launch_setup(context, *args, **kwargs):
    # fmt: off
    architecture_type                           = LaunchConfiguration("architecture_type",                           default="awf/universe/20240605")
    autoware_launch_file                        = LaunchConfiguration("autoware_launch_file",                        default=default_autoware_launch_file_of(architecture_type.perform(context)))
    autoware_launch_package                     = LaunchConfiguration("autoware_launch_package",                     default=default_autoware_launch_package_of(architecture_type.perform(context)))
    consider_acceleration_by_road_slope         = LaunchConfiguration("consider_acceleration_by_road_slope",         default=False)
    consider_pose_by_road_slope                 = LaunchConfiguration("consider_pose_by_road_slope",                 default=True)
    enable_perf                                 = LaunchConfiguration("enable_perf",                                 default=False)
    enable_stuck_jump                           = LaunchConfiguration("enable_stuck_jump",                           default=True)
    global_frame_rate                           = LaunchConfiguration("global_frame_rate",                           default=30.0)
    global_real_time_factor                     = LaunchConfiguration("global_real_time_factor",                     default=1.0)
    global_timeout                              = LaunchConfiguration("global_timeout",                              default=180)
    initialize_duration                         = LaunchConfiguration("initialize_duration",                         default=30)
    initialize_localization                     = LaunchConfiguration("initialize_localization",                     default=10)
    launch_autoware                             = LaunchConfiguration("launch_autoware",                             default=True)
    launch_rviz                                 = LaunchConfiguration("launch_rviz",                                 default=False)
    launch_simple_sensor_simulator              = LaunchConfiguration("launch_simple_sensor_simulator",              default=True)
    launch_visualization                        = LaunchConfiguration("launch_visualization",                        default=True)
    output_directory                            = LaunchConfiguration("output_directory",                            default=Path("/tmp"))
    override_parameters                         = LaunchConfiguration("override_parameters",                         default="")
    parameter_file_path                         = LaunchConfiguration("parameter_file_path",                         default=Path(get_package_share_directory("scenario_test_runner")) / "config/parameters.yaml")
    pedestrian_ignore_see_around                = LaunchConfiguration("pedestrian_ignore_see_around",                default="blind")
    port                                        = LaunchConfiguration("port",                                        default=5555)
    publish_empty_context                       = LaunchConfiguration("publish_empty_context",                       default=False)
    record                                      = LaunchConfiguration("record",                                      default=True)
    record_option                               = LaunchConfiguration("record_option",                               default="")
    record_storage_id                           = LaunchConfiguration("record_storage_id",                           default="")
    rviz_config                                 = LaunchConfiguration("rviz_config",                                 default=default_rviz_config_file())
    scenario                                    = LaunchConfiguration("scenario",                                    default=Path("/dev/null"))
    sensor_model                                = LaunchConfiguration("sensor_model",                                default="")
    sigterm_timeout                             = LaunchConfiguration("sigterm_timeout",                             default=8)
    simulate_localization                       = LaunchConfiguration("simulate_localization",                       default=True)
    speed_condition                             = LaunchConfiguration("speed_condition",                             default="legacy")
    status_monitor_threshold                    = LaunchConfiguration("status_monitor_threshold",                    default=10)
    stuck_jump_distance                         = LaunchConfiguration("stuck_jump_distance",                         default=0.1)
    stuck_jump_timeout                          = LaunchConfiguration("stuck_jump_timeout",                          default=7.0)
    stuck_speed_threshold                       = LaunchConfiguration("stuck_speed_threshold",                       default=0.1)
    trajectory_based_detection_offset           = LaunchConfiguration("trajectory_based_detection_offset",           default=0.0)
    use_custom_centerline                       = LaunchConfiguration("use_custom_centerline",                       default=True)
    use_sim_time                                = LaunchConfiguration("use_sim_time",                                default=False)
    use_trajectory_based_front_entity_detection = LaunchConfiguration("use_trajectory_based_front_entity_detection", default=False)
    vehicle_model                               = LaunchConfiguration("vehicle_model",                               default="")
    vehicle_id                                  = LaunchConfiguration("vehicle_id",                                  default="default")
    # Godot simulator options
    godot_executable                            = LaunchConfiguration("godot_executable",                            default="/home/kotaroyoshimoto/Downloads/godot_autoware_simulator.x86_64")
    # fmt: on
    vehicle_model_name = vehicle_model.perform(context)
    use_godot_sim = vehicle_model_name.endswith("_godot")
    if use_godot_sim:
        vehicle_model_name = vehicle_model_name[: -len("_godot")]
    godot_executable_path = ""
    if use_godot_sim:
        executable_str = godot_executable.perform(context)
        if not executable_str:
            raise ValueError(
                "godot_executable must be specified when vehicle_model ends with '_godot'. "
                "Pass godot_executable:=/path/to/godot_autoware_simulator.x86_64"
            )
        godot_executable_path = Path(executable_str)
        if godot_executable_path.is_absolute() or godot_executable_path.parent != Path("."):
            if godot_executable_path.is_file():
                godot_executable_path = str(godot_executable_path)
            else:
                raise FileNotFoundError(f'Executable "{godot_executable_path}" does not exist.')
        elif resolved := shutil.which(godot_executable_path):
            godot_executable_path = resolved
        else:
            raise FileNotFoundError(f'Executable "{godot_executable_path}" could not be resolved from PATH.')

    print(f"architecture_type                           := {architecture_type.perform(context)}")
    print(f"autoware_launch_file                        := {autoware_launch_file.perform(context)}")
    print(f"autoware_launch_package                     := {autoware_launch_package.perform(context)}")
    print(f"consider_acceleration_by_road_slope         := {consider_acceleration_by_road_slope.perform(context)}")
    print(f"consider_pose_by_road_slope                 := {consider_pose_by_road_slope.perform(context)}")
    print(f"enable_perf                                 := {enable_perf.perform(context)}")
    print(f"enable_stuck_jump                           := {enable_stuck_jump.perform(context)}")
    print(f"global_frame_rate                           := {global_frame_rate.perform(context)}")
    print(f"global_real_time_factor                     := {global_real_time_factor.perform(context)}")
    print(f"global_timeout                              := {global_timeout.perform(context)}")
    print(f"initialize_duration                         := {initialize_duration.perform(context)}")
    print(f"initialize_localization                     := {initialize_localization.perform(context)}")
    print(f"launch_autoware                             := {launch_autoware.perform(context)}")
    print(f"launch_rviz                                 := {launch_rviz.perform(context)}")
    print(f"launch_visualization                        := {launch_visualization.perform(context)}")
    print(f"output_directory                            := {output_directory.perform(context)}")
    print(f"override_parameters                         := {override_parameters.perform(context)}")
    print(f"parameter_file_path                         := {parameter_file_path.perform(context)}")
    print(f"pedestrian_ignore_see_around                := {pedestrian_ignore_see_around.perform(context)}")
    print(f"port                                        := {port.perform(context)}")
    print(f"publish_empty_context                       := {publish_empty_context.perform(context)}")
    print(f"record                                      := {record.perform(context)}")
    print(f"record_option                               := {record_option.perform(context)}")
    print(f"record_storage_id                           := {record_storage_id.perform(context)}")
    print(f"rviz_config                                 := {rviz_config.perform(context)}")
    print(f"scenario                                    := {scenario.perform(context)}")
    print(f"sensor_model                                := {sensor_model.perform(context)}")
    print(f"sigterm_timeout                             := {sigterm_timeout.perform(context)}")
    print(f"simulate_localization                       := {simulate_localization.perform(context)}")
    print(f"speed_condition                             := {speed_condition.perform(context)}")
    print(f"status_monitor_threshold                    := {status_monitor_threshold.perform(context)}")
    print(f"stuck_jump_distance                         := {stuck_jump_distance.perform(context)}")
    print(f"stuck_jump_timeout                          := {stuck_jump_timeout.perform(context)}")
    print(f"stuck_speed_threshold                       := {stuck_speed_threshold.perform(context)}")
    print(f"trajectory_based_detection_offset           := {trajectory_based_detection_offset.perform(context)}")
    print(f"use_custom_centerline                       := {use_custom_centerline.perform(context)}")
    print(f"use_sim_time                                := {use_sim_time.perform(context)}")
    print(f"use_trajectory_based_front_entity_detection := {use_trajectory_based_front_entity_detection.perform(context)}")
    print(f"vehicle_model                               := {vehicle_model_name}")
    print(f"vehicle_id                                  := {vehicle_id.perform(context)}")

    def make_launch_prefix():
        if enable_perf.perform(context) == "True":
            return "perf record -F 10000"
        else:
            return ""

    def collect_prefixed_launch_configurations(prefix):
        return [
            (key[len(prefix):], value)
            for key, value in context.launch_configurations.items()
            if key.startswith(prefix)
        ]

    def to_typed_scalar(value):
        if value.lower() == "true":
            return True
        if value.lower() == "false":
            return False
        for convert in (int, float):
            try:
                return convert(value)
            except ValueError:
                pass
        return value

    def make_simple_sensor_simulator_parameters():
        return [{key: to_typed_scalar(value)} for key, value in collect_prefixed_launch_configurations("simple_sensor_simulator.")]

    def make_parameters():
        parameters = [
            {"architecture_type": architecture_type},
            {"autoware_launch_file": autoware_launch_file},
            {"autoware_launch_package": autoware_launch_package},
            {"consider_acceleration_by_road_slope": consider_acceleration_by_road_slope},
            {"consider_pose_by_road_slope": consider_pose_by_road_slope},
            {"enable_stuck_jump": enable_stuck_jump},
            {"initialize_duration": initialize_duration},
            {"initialize_localization": initialize_localization},
            {"launch_autoware": launch_autoware},
            {"pedestrian_ignore_see_around": pedestrian_ignore_see_around},
            {"port": port},
            {"publish_empty_context" : publish_empty_context},
            {"record": record},
            {"record_option": record_option},
            {"record_storage_id": record_storage_id},
            {"rviz_config": rviz_config},
            {"sensor_model": sensor_model},
            {"sigterm_timeout": sigterm_timeout},
            {"simulate_localization": simulate_localization},
            {"speed_condition": speed_condition},
            {"status_monitor_threshold": status_monitor_threshold},
            {"stuck_jump_distance": stuck_jump_distance},
            {"stuck_jump_timeout": stuck_jump_timeout},
            {"stuck_speed_threshold": stuck_speed_threshold},
            {"trajectory_based_detection_offset": trajectory_based_detection_offset},
            {"use_custom_centerline": use_custom_centerline},
            {"use_sim_time": use_sim_time},
            {"use_trajectory_based_front_entity_detection": use_trajectory_based_front_entity_detection},
            {"vehicle_model": vehicle_model_name},
            {"vehicle_id": vehicle_id},
        ]

        def collect_vehicle_parameters():
            if vehicle_model_name:
                description = get_package_share_directory(vehicle_model_name + "_description")
                if use_godot_sim:
                    return [
                        description + "/config/vehicle_info.param.yaml",
                        {"vehicle_model_type": "EXTERNAL"},
                        # Disable all concealer publishers (replaces group-level publish_localization / publish_vehicle_state)
                        # Localization publishers (simulate_localization=True topics)
                        {"/localization/acceleration.enabled": False},
                        {"/localization/kinematic_state.enabled": False},
                        {"/simulation/debug/localization/pose_estimator/pose_with_covariance.enabled": False},
                        # Vehicle state publishers
                        {"/vehicle/status/steering_status.enabled": False},
                        {"/vehicle/status/gear_status.enabled": False},
                        {"/vehicle/status/control_mode.enabled": False},
                        {"/vehicle/status/velocity_status.enabled": False},
                        {"/vehicle/status/turn_indicators_status.enabled": False},
                        # TF broadcaster
                        {"tf.enabled": False},
                        # Service servers (Godot advertises these via rosbridge instead)
                        {"/control/control_mode_request.enabled": False},
                    ]
                else:
                    return [
                        description + "/config/vehicle_info.param.yaml",
                        description + "/config/simulator_model.param.yaml",
                    ]
            else:
                return []

        if (it := collect_vehicle_parameters()) != []:
            parameters += it

        def format_autoware_parameters(items):
            return [key + ":=" + value for key, value in items]

        if (it := format_autoware_parameters(collect_prefixed_launch_configurations("autoware."))) != []:
            parameters += [{"autoware.": it}]

        path = Path(parameter_file_path.perform(context))

        if not path.is_file():
            raise Exception(f'The value "{path}" given for parameter `parameter_file_path` is not a file.')
        elif path.suffix not in {'.yaml', '.yml'}:
            raise Exception(f'The value "{path}" given for parameter `parameter_file_path` is not a YAML file.')
        else:
            parameters += [path]

        return parameters

    def make_agnocast_additional_environment():
        if os.getenv('ENABLE_AGNOCAST', '') == '1':
            return {
                'LD_PRELOAD': f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
                'AGNOCAST_MEMPOOL_SIZE': '134217728',
            }
        else:
            return {}

    return [
        # fmt: off
        DeclareLaunchArgument("architecture_type",                           default_value=architecture_type                          ),
        DeclareLaunchArgument("autoware_launch_file",                        default_value=autoware_launch_file                       ),
        DeclareLaunchArgument("autoware_launch_package",                     default_value=autoware_launch_package                    ),
        DeclareLaunchArgument("consider_acceleration_by_road_slope",         default_value=consider_acceleration_by_road_slope        ),
        DeclareLaunchArgument("consider_pose_by_road_slope",                 default_value=consider_pose_by_road_slope                ),
        DeclareLaunchArgument("enable_perf",                                 default_value=enable_perf                                ),
        DeclareLaunchArgument("enable_stuck_jump",                           default_value=enable_stuck_jump                          ),
        DeclareLaunchArgument("global_frame_rate",                           default_value=global_frame_rate                          ),
        DeclareLaunchArgument("global_real_time_factor",                     default_value=global_real_time_factor                    ),
        DeclareLaunchArgument("global_timeout",                              default_value=global_timeout                             ),
        DeclareLaunchArgument("initialize_localization",                     default_value=initialize_localization                    ),
        DeclareLaunchArgument("launch_autoware",                             default_value=launch_autoware                            ),
        DeclareLaunchArgument("launch_rviz",                                 default_value=launch_rviz                                ),
        DeclareLaunchArgument("output_directory",                            default_value=output_directory                           ),
        DeclareLaunchArgument("parameter_file_path",                         default_value=parameter_file_path                        ),
        DeclareLaunchArgument("pedestrian_ignore_see_around",                default_value=pedestrian_ignore_see_around               ),
        DeclareLaunchArgument("publish_empty_context",                       default_value=publish_empty_context                      ),
        DeclareLaunchArgument("record_option",                               default_value=record_option                              ),
        DeclareLaunchArgument("rviz_config",                                 default_value=rviz_config                                ),
        DeclareLaunchArgument("scenario",                                    default_value=scenario                                   ),
        DeclareLaunchArgument("sensor_model",                                default_value=sensor_model                               ),
        DeclareLaunchArgument("sigterm_timeout",                             default_value=sigterm_timeout                            ),
        DeclareLaunchArgument("simulate_localization",                       default_value=simulate_localization                      ),
        DeclareLaunchArgument("speed_condition",                             default_value=speed_condition                            ),
        DeclareLaunchArgument("status_monitor_threshold",                    default_value=status_monitor_threshold                   ),
        DeclareLaunchArgument("stuck_jump_distance",                         default_value=stuck_jump_distance                        ),
        DeclareLaunchArgument("stuck_jump_timeout",                          default_value=stuck_jump_timeout                         ),
        DeclareLaunchArgument("stuck_speed_threshold",                       default_value=stuck_speed_threshold                      ),
        DeclareLaunchArgument("trajectory_based_detection_offset",           default_value=trajectory_based_detection_offset          ),
        DeclareLaunchArgument("use_custom_centerline",                       default_value=use_custom_centerline                      ),
        DeclareLaunchArgument("use_sim_time",                                default_value=use_sim_time                               ),
        DeclareLaunchArgument("use_trajectory_based_front_entity_detection", default_value=use_trajectory_based_front_entity_detection),
        DeclareLaunchArgument("vehicle_model",                               default_value=vehicle_model                              ),
        DeclareLaunchArgument("godot_executable",                            default_value=godot_executable                           ),
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
                "--override-parameters",     override_parameters,
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
            parameters=make_parameters() + make_simple_sensor_simulator_parameters(),
            condition=IfCondition(launch_simple_sensor_simulator),
            additional_env=make_agnocast_additional_environment(),
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
            condition=IfCondition(launch_visualization),
            output="screen",
            parameters=make_parameters(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output={"stderr": "log", "stdout": "log"},
            condition=IfCondition(launch_rviz),
            arguments=["-d", str(default_rviz_config_file())],
        ),
    ] + (
        [
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    )
                ),
                launch_arguments={
                    "port": "9090",
                    "max_message_size": "50000000",
                    "call_services_in_new_thread": "true",
                }.items(),
            ),
            Node(
                package="scenario_test_runner",
                executable="lanelet_bridge_node.py",
                name="lanelet_bridge_node",
                output="screen",
                on_exit=ShutdownOnce(),
            ),
            ExecuteProcess(
                cmd=[
                    godot_executable_path,
                    "--headless",
                    "--vehicle-params",
                    str(
                        Path(get_package_share_directory("scenario_test_runner"))
                        / "config/sakoda_vehicle_params.json"
                    ),
                ],
                output="screen",
            ),
        ]
        if use_godot_sim
        else []
    )


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
