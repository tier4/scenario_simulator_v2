#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for random test runner."""

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
#
# Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def architecture_types():
    return ["awf/auto", "awf/universe", "awf/universe/20230906", "tier4/proposal"]


def default_autoware_launch_package_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type:={architecture_type.perform(context)} is not supported. Choose one of {architecture_types()}."
        )

    return {
        "awf/auto": "scenario_simulator_launch",
        "awf/universe": "autoware_launch",
        "awf/universe/20230906": "autoware_launch",
        "tier4/proposal": "autoware_launch",
    }[architecture_type]


def default_autoware_launch_file_of(architecture_type):
    if architecture_type not in architecture_types():
        raise KeyError(
            f"architecture_type:={architecture_type.perform(context)} is not supported. Choose one of {architecture_types()}."
        )

    return {
        "awf/auto": "autoware_auto.launch.py",
        "awf/universe": "planning_simulator.launch.xml",
        "awf/universe/20230906": "planning_simulator.launch.xml",
        "tier4/proposal": "planning_simulator.launch.xml",
    }[architecture_type]


class RandomTestRunnerLaunch(object):
    def __init__(self):
        self.random_test_runner_launch_configuration = {}
        self.autoware_launch_configuration = {}

        self.autoware_launch_arguments = {
            # autoware arguments #
            "architecture_type": {"default": "awf/auto", "description": "Autoware architecture type", "values": architecture_types()},
            "sensor_model": {"default": "aip_xx1", "description": "Ego sensor model"},
            "vehicle_model": {"default": "lexus", "description": "Ego vehicle model"},
        }

        self.random_test_arguments = {
            "test_parameters_filename":
                {"default": "",
                 "description": "Yaml filename within random_test_runner/param directory containing test parameters."
                                "If specified (not empty), other test arguments will be ignored"},
            "simulator_type": {"default": "simple_sensor_simulator", "description": "Simulation backend",
                               "values": ["simple_sensor_simulator", "unity"]},
            "simulator_host":
                {"default": "localhost",
                 "description": "Simulation host. It can be either IP address "
                                "or the host name that is resolvable in the environment"},

            # control arguments #
            "test_count": {"default": 5, "description": "Test count to be performed in test suite"},
            "input_dir":
                {"default": "",
                 "description": "Directory containing the result.yaml file to be replayed. "
                                "If not empty, tests will be replayed from result.yaml"},
            "output_dir":
                {"default": "/tmp",
                 "description": "Directory to which result.yaml and result.junit.xml files will be placed"},

            "initialize_duration": {"default": 35, "description": "How long test runner will wait for Autoware to initialize"},

            # test suite arguments #
            "test_name": {"default": "random_test",
                          "description": "Test name. Used for descriptive purposes only"},
            "map_name": {"default": "kashiwanoha_map",
                         "description": "Package name containing map information (lanelet, point cloud, etc)"},
            "ego_goal_lanelet_id":
                {"default": -1,
                 "description": "Goal lanelet id. If -1, goal will be chosen randomly"},
            "ego_goal_s":
                {"default": 0.0,
                 "description": "Goal lanelet s (translation along the lanelet in meters). "
                                "If ego_goal_lanelet_id equals -1, s will be chosen randomly"},
            "ego_goal_partial_randomization":
                {"default": False,
                 "description": "If true, goal will be randomized with distance set in "
                                "ego_goal_partial_randomization_distance value. If ego_goal_lanelet_id is set to -1, "
                                "this value is ignored"},
            "ego_goal_partial_randomization_distance":
                {"default": 25.0,
                 "description": "Distance from goal set by ego_goal_lanelet_id and ego_goal_s, within which goal "
                                "pose will be randomized if ego_goal_partial_randomization is set to true"},
            "npc_count": {"default": 10, "description": "Generated npc count"},
            "npc_min_speed": {"default": 0.5, "description": "Minimum speed of generated npcs"},
            "npc_max_speed": {"default": 3.0, "description": "Maximum speed of generated npcs"},
            "npc_min_distance_to_ego": {"default": 10.0,
                                        "description":  "Minimum distance of generated npcs from ego"},
            "npc_max_distance_to_ego": {"default": 100.0,
                                        "description": "Maximum distance of generated npcs from ego"},

            # test case arguments #
            "seed": {"default": -1, "description": "Randomization seed. If -1, seed will be generated for each test"},
        }

    def collect_random_test_launch_configuration(self):
        for argument_name, argument_default_and_description in self.random_test_arguments.items():
            self.random_test_runner_launch_configuration[argument_name] = \
                LaunchConfiguration(argument_name, default=argument_default_and_description["default"])

        for argument_name, argument_default_and_description in self.autoware_launch_arguments.items():
            self.autoware_launch_configuration[argument_name] = \
                LaunchConfiguration(argument_name, default=argument_default_and_description["default"])

    def create_declared_launch_argument(self, name, configuration):
        argument_possible_values = None
        if "values" in configuration.keys():
            argument_possible_values = configuration["values"]
            print("Value of {}: {}".format(name, argument_possible_values))

        return DeclareLaunchArgument(
            name,
            default_value=str(configuration["default"]),
            description=configuration["description"],
            choices=argument_possible_values
        )

    def generate_launch_arguments_declarations(self):
        declared_launch_arguments = []

        for configuration_name, configuration_value in self.random_test_runner_launch_configuration.items():
            declared_launch_arguments.append(
                self.create_declared_launch_argument(configuration_name,
                                                     self.random_test_arguments[configuration_name]))

        for configuration_name, configuration_value in self.autoware_launch_configuration.items():
            declared_launch_arguments.append(
                self.create_declared_launch_argument(configuration_name,
                                                     self.autoware_launch_arguments[configuration_name]))

        return declared_launch_arguments

    def launch_setup(self, context, *args, **kwargs):
        test_param_file = self.random_test_runner_launch_configuration["test_parameters_filename"].perform(context)
        print("Test param file '{}'".format(test_param_file))

        autoware_architecture = self.autoware_launch_configuration["architecture_type"].perform(context)
        print("Autoware architecture '{}'".format(autoware_architecture))

        parameters = [self.autoware_launch_configuration,
                      {"autoware_launch_package": default_autoware_launch_package_of(autoware_architecture),
                       "autoware_launch_file": default_autoware_launch_file_of(autoware_architecture)},
                      self.random_test_runner_launch_configuration]

        if test_param_file:
            test_param_file_path = os.path.join(get_package_share_directory("random_test_runner"), "param",
                                                test_param_file)
            print("Parameters file supplied: '{}'. "
                  "Parameters passed there override passed via arguments".format(test_param_file_path))
            parameters.append(test_param_file_path)

        # not tested for other architectures but required for "awf/universe"
        if "awf/universe" in autoware_architecture:
            vehicle_model = self.autoware_launch_configuration["vehicle_model"].perform(context)
            if vehicle_model:
                vehicle_model_description_dir = get_package_share_directory(vehicle_model + "_description")

                vehicle_info_param_file_path = os.path.join(vehicle_model_description_dir, "config/vehicle_info.param.yaml")
                simulator_model_param_file_path = os.path.join(vehicle_model_description_dir, "config/simulator_model.param.yaml")

                print("Vehicle info parameters file supplied: '{}'. "
                    "Parameters passed there override passed via arguments".format(vehicle_info_param_file_path))
                print("Simulator model parameters file supplied: '{}'. "
                    "Parameters passed there override passed via arguments".format(simulator_model_param_file_path))
                parameters.append(vehicle_info_param_file_path)
                parameters.append(simulator_model_param_file_path)

        scenario_node = Node(
            package="random_test_runner",
            executable="random_test_runner",
            namespace="simulation",
            name="random_test_runner",
            output="screen",
            arguments=[("__log_level:=info")],
            parameters=parameters
        )

        shutdown_handler = OnProcessExit(
            target_action=scenario_node, on_exit=[EmitEvent(event=Shutdown())]
        )

        launch_description = [
            scenario_node,
            RegisterEventHandler(event_handler=shutdown_handler),
            Node(
                package="openscenario_visualization",
                executable="openscenario_visualization_node",
                namespace="simulation",
                name="openscenario_visualizer",
                output="screen",
            ),
        ]

        simulation_type = self.random_test_runner_launch_configuration["simulator_type"].perform(context)
        print("Chosen simulation type", simulation_type)
        if simulation_type == "simple_sensor_simulator":
            launch_description.append(
                Node(
                    package="simple_sensor_simulator",
                    executable="simple_sensor_simulator_node",
                    name="simple_sensor_simulator_node",
                    namespace="simulation",
                    output="log",
                    arguments=[("__log_level:=warn")],
                    parameters=[{"port": 5555}],
                ),
            )

        return launch_description


def generate_launch_description():
    launch = RandomTestRunnerLaunch()
    launch.collect_random_test_launch_configuration()
    launch_description_list = [OpaqueFunction(function=launch.launch_setup)]
    launch_description_list.extend(launch.generate_launch_arguments_declarations())
    return LaunchDescription(launch_description_list)
