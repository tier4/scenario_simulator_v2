import os
import sys
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout

import pytest


def generate_mock_base_command():
    return [
        "ros2",
        "launch",
        "cpp_mock_scenarios",
        "mock_base.launch.py"]


def generate_scenario_command():
    return [
        "ros2",
        "run",
        "idiot_npc"]


@ pytest.mark.launch_test
def generate_test_description():
    simulator_process = launch.actions.ExecuteProcess(
        cmd=generate_mock_base_command(),
        output='screen'
    )
    scenario_process = launch.actions.ExecuteProcess(
        cmd=generate_scenario_command(),
        output='screen'
    )
    return launch.LaunchDescription(
        [simulator_process, simulator_process])
