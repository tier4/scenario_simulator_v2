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
import launch
import launch_ros
import lifecycle_msgs

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from pathlib import Path


def generate_launch_description():
    global_frame_rate = LaunchConfiguration('global-frame-rate', default=30.0)
    global_real_time_factor = LaunchConfiguration('global-real-time-factor', default=1.0)
    global_timeout = LaunchConfiguration('global-timeout', default=180)
    output_directory = LaunchConfiguration('output-directory', default=Path("/tmp"))
    scenario = LaunchConfiguration('scenario', default=Path("/dev/null"))  # NOTE: DON'T USE 'None'
    workflow = LaunchConfiguration('workflow', default=Path("/dev/null"))  # NOTE: DON'T USE 'None'

    port = 8080

    interpreter = LifecycleNode(
        package='openscenario_interpreter',
        executable='openscenario_interpreter_node',
        namespace='simulation',
        name='openscenario_interpreter',
        output='screen',
        parameters=[{
            'map_path': os.path.join(
                get_package_share_directory('kashiwanoha_map'), 'map', 'lanelet2_map.osm'),
            'origin_latitude':   34.903555800615614,
            'origin_longitude': 139.93339979022568,
            'port': port,
            }],)

    launch_autoware = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(
            get_package_share_directory('scenario_test_runner') +
            '/autoware.launch.xml'))

    return LaunchDescription([
        DeclareLaunchArgument('global-frame-rate', default_value=global_frame_rate),
        DeclareLaunchArgument('global-real-time-factor', default_value=global_real_time_factor),
        DeclareLaunchArgument('global-timeout', default_value=global_timeout),
        DeclareLaunchArgument('output-directory', default_value=output_directory),
        DeclareLaunchArgument('scenario', default_value=scenario),
        DeclareLaunchArgument('workflow', default_value=workflow),

        Node(
            package='scenario_test_runner',
            executable='scenario_test_runner',
            namespace='simulation',
            name='scenario_test_runner',
            output='screen',
            on_exit=Shutdown(),
            arguments=[
                '--global-frame-rate', global_frame_rate,
                '--global-real-time-factor', global_real_time_factor,
                '--global-timeout', global_timeout,
                '--output-directory', output_directory,
                '--scenario', scenario,
                '--workflow', workflow,],),

        Node(
            package='scenario_simulator',
            executable='scenario_simulator_node',
            namespace='simulation',
            name='sensor_simulator',
            output='log',
            parameters=[{
                'port': port,
                }],),

        interpreter,

        # launch.actions.EmitEvent(
        #     event=launch_ros.events.lifecycle.ChangeState(
        #         lifecycle_node_matcher=launch.events.matches_action(interpreter),
        #         # transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        #         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        #         )),
        #
        # launch.actions.EmitEvent(
        #     event=launch_ros.events.lifecycle.ChangeState(
        #         lifecycle_node_matcher=launch.events.matches_action(interpreter),
        #         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP,)),

        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=interpreter,
                # start_state = 'configuring', goal_state='inactive',
                # goal_state='configuring',
                # start_state='inactive', goal_state='activating',
                # start_state='activating', goal_state='active',
                goal_state='active',
                entities=[
                   launch.actions.LogInfo(msg="activating interpreter"),
                   launch_autoware,],)),

         launch.actions.RegisterEventHandler(
             launch_ros.event_handlers.OnStateTransition(
                 target_lifecycle_node=interpreter,
                 goal_state='deactivate',
                 entities=[
                       launch.actions.LogInfo(msg="deactivating interpreter"),
                       # launch.actions.EmitEvent(event=launch.events.Shutdown()),
                   ],)),

        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=interpreter,
                goal_state='finalized',
                entities=[
                    launch.actions.LogInfo(msg="finalizing interpreter"),
                    launch.actions.EmitEvent(event=launch.events.Shutdown()),],))

        Node(
            package='openscenario_visualization',
            executable='openscenario_visualization_node',
            namespace='simulation',
            name='openscenario_visualizer',
            output='log',),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output={
        #         'stderr': 'log',
        #         'stdout': 'log',
        #         },
        #     arguments=[
        #         # '-d', os.path.join(
        #         #     get_package_share_directory('simulation_api'), 'config/moc_test.rviz')
        #         '-d', str(
        #             Path(get_package_share_directory('autoware_launch')) / 'rviz/autoware.rviz')
        #         ],
        #     ),

        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(
        #         get_package_share_directory('scenario_test_runner') + '/autoware.launch.xml'))
        ])
