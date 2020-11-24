"""Generate launch description for scenario runner."""

# Copyright 2015-2020 TierIV.inc. All rights reserved.
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

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    """Generate launch description for scenario runner."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='openscenario_interpreter',
            node_executable='openscenario_interpreter_node',
            node_name='openscenario_interpreter',
            output='screen',
            parameters=[{
                'verbose': 'true',
                'scenario': os.path.join(
                    ament_index_python.packages.get_package_share_directory(
                        'openscenario_interpreter'),
                    'example/lane_change.xosc')}])])
