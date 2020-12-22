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

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workflow = LaunchConfiguration('workflow')

    declare_workflow = DeclareLaunchArgument(
        'workflow',
        default_value=workflow,
        description='workflow files for scenario testing')

    log_directory = LaunchConfiguration('log_directory', default="/tmp")

    declare_log_directory = DeclareLaunchArgument(
        'log_directory',
        default_value=log_directory,
        description='log_directory files for scenario testing')

    no_validation = LaunchConfiguration('no_validation', default=False)

    declare_no_validation = DeclareLaunchArgument(
        'no_validation',
        default_value=no_validation
        )

    # NOTE: https://answers.ros.org/question/332829/no-stdout-logging-output-in-ros2-using-launch/
    scenario_test_runner = Node(
        package='scenario_test_runner',
        node_executable='scenario_test_runner',
        output={
            'stdout': 'screen',  # THIS OPTION NOT WORKS IF (< ROS2 ELOQUENT)
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
        arguments=[
            "--log_directory", log_directory,
            "--no_validation", no_validation,
            "--workflow", workflow,
        ]
    )

    port = 8080

    scenario_simulator = Node(
        package='scenario_simulator',
        node_executable='scenario_simulator_node',
        node_name='scenario_simulator_node',
        output='log',
        parameters=[{
            'port': port,
        }],
        arguments=[('__log_level:=warn')],
    )

    openscenario_interpreter = LifecycleNode(
        package='openscenario_interpreter',
        node_executable='openscenario_interpreter_node',
        node_name='openscenario_interpreter_node',
        output='screen',
        parameters=[{
            'map_path': os.path.join(
                get_package_share_directory('kashiwanoha_map'), 'map', 'lanelet2_map.osm'),
            'origin_latitude': 34.903555800615614,
            'origin_longitude': 139.93339979022568,
            'port': port,
        }]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('simulation_api'), 'config/moc_test.rviz')
            ],
        output='log'
    )

    openscenario_visualization = Node(
        package='openscenario_visualization',
        node_executable='openscenario_visualization_node',
        node_name='openscenario_visualization_node',
        output='log'
    )

    description = LaunchDescription()
    description.add_action(declare_log_directory)
    description.add_action(declare_no_validation)
    description.add_action(declare_workflow)
    description.add_action(openscenario_interpreter)
    description.add_action(openscenario_visualization)
    description.add_action(rviz2)
    description.add_action(scenario_simulator)
    description.add_action(scenario_test_runner)

    return description
