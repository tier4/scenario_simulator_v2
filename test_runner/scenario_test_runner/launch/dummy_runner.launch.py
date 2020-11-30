#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    scenario_test_runner = Node(
        package='scenario_test_runner',
        node_executable='scenario_test_runner',
        output={
                'stdout': 'log',
                'stderr': 'screen',
        }
    )
    scenario_runner_mock = LifecycleNode(
        node_name='scenario_runner_mock',
        package='scenario_runner_mock',
        node_executable='scenario_runner_mock',
        output='log'
    )
    return LaunchDescription([scenario_test_runner, scenario_runner_mock])
