#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Autoware Foundation. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
import launch

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    workflow = LaunchConfiguration('workflow')
    declare_workflow = DeclareLaunchArgument(
                'workflow',
                default_value=workflow,
                description='workflow files for scenario testing')
    scenario_test_runner = Node(
        package='scenario_test_runner',
        node_executable='scenario_test_runner',
        output={
                'stdout': 'log',
                'stderr': 'screen',
        },
        on_exit=launch.actions.Shutdown(),
        arguments=["--workflow", workflow]
    )
    open_scenario_interpreter = LifecycleNode(
        node_name='open_scenario_interpreter_node',
        package='open_scenario_interpreter',
        node_executable='open_scenario_interpreter_node',
        output='screen'
    )
    description = LaunchDescription()
    description.add_action(declare_workflow)
    description.add_action(scenario_test_runner)
    description.add_action(open_scenario_interpreter)
    return description
