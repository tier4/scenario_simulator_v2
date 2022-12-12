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
from ament_index_python.packages import get_package_share_directory


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


def test_launch():
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
        ],
        "Something wrong while running scenarios, please check test result file in /tmp/scenario_test_runner/result.junit.xml",
    )
    run_bash_command(
        [
            "ros2",
            "run",
            "scenario_test_runner",
            "result_checker",
            "/tmp/scenario_test_runner/result.junit.xml",
        ],
        "Some of the scenario was failed, please check test result file in /tmp/scenario_test_runner/result.junit.xml",
    )
