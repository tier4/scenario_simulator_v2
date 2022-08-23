#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for spline visualization."""

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

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for spline visualization."""
    return LaunchDescription(
        [
            Node(
                package="simulation_interface",
                executable="example",
                name="example",
                output="screen",
                arguments=[("__log_level:=info")],
            )
        ]
    )
