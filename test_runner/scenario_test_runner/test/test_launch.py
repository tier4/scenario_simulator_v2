# Copyright 2022 TIER IV, Inc.
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

import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import shutil


def run_bash_command(bash_commands, error_message):
    cmd = ""
    for i, command in enumerate(bash_commands):
        if i == 0:
            cmd = command
        else:
            cmd = cmd + " " + command
    execute_cmd = ["/bin/bash", "-c", cmd]
    result = subprocess.run(
        execute_cmd, shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if result.returncode != 0:
        assert result == 0, error_message


def get_output_path(package_name: str = "scenario_test_runner"):
    return os.path.join(
        get_package_prefix(package_name),
        "..",
        "..",
        "build",
        package_name,
        "scenario_tests",
    )


def make_output_directory(directory: str):
    if not os.path.exists(directory):
        os.makedirs(directory)
    else:
        shutil.rmtree(directory)
        os.makedirs(directory)


def test_launch():
    make_output_directory(get_output_path())
    run_bash_command(
        [
            "ros2",
            "launch",
            "scenario_test_runner",
            "scenario_test_runner.launch.py",
            "workflow:="
            + get_package_share_directory("scenario_test_runner")
            + "/config/workflow_example.yaml",
            "global_frame_rate:=20",
            "output_directory:=" + get_output_path(),
        ],
        "Something wrong while running scenarios, please check test result file in /tmp/scenario_test_runner/result.junit.xml",
    )
    run_bash_command(
        [
            "ros2",
            "run",
            "scenario_test_runner",
            "result_checker",
            os.path.join(get_output_path(), "/scenario_test_runner/result.junit.xml"),
        ],
        "Some of the scenarios were failed, please check test result file in /tmp/scenario_test_runner/result.junit.xml",
    )
