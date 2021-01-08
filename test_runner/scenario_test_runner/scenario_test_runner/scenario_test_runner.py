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


import argparse
# import rclpy
import time

from openscenario_utility.conversion import convert
from openscenario_utility.validation import XOSCValidator
from pathlib import Path
from scenario_test_runner.lifecycle_controller import LifecycleController
from scenario_test_runner.workflow import Scenario, Workflow, substitute_ros_package
from shutil import rmtree
from sys import exit
from typing import List


def convert_scenarios(scenarios: List[Scenario], output_directory: Path):

    result = []

    for each in scenarios:

        if each.path.suffix == '.xosc':

            result.append(each)

        else:  # == '.yaml'

            for path in convert(
                    each.path,  # input
                    output_directory / each.path.stem,  # output
                    False
                    ):

                result.append(
                    Scenario(path, each.expect, each.frame_rate))

    return result


class ScenarioTestRunner(LifecycleController):
    """
    Class to test scenarios.

    Attributes
    ----------
    SLEEP_RATE : int
        Time to sleep before next scenario.

    """

    SLEEP_RATE = 1

    def __init__(
            self,  # Arguments are alphabetically sorted
            global_frame_rate: float,
            global_real_time_factor: float,
            global_timeout: int,  # [sec]
            output_directory: Path,
            ):
        """
        Initialize the class ScenarioTestRunner.

        Arguments
        ---------
        global_timeout : int
            If the success or failure of the simulation is not determined even
            after the specified time (seconds) has passed, the simulation is
            forcibly terminated as a failure.

        output_directory : Path
            Output destination directory of the generated file including the
            result file.

        Returns
        -------
        None

        """
        self.global_frame_rate = global_frame_rate
        self.global_real_time_factor = global_real_time_factor
        self.global_timeout = global_timeout

        self.launcher_path = Path(__file__).resolve().parent.parent

        self.output_directory = substitute_ros_package(output_directory)

        if self.output_directory.exists():
            rmtree(self.output_directory)

        self.output_directory.mkdir(parents=True, exist_ok=True)

        self.xosc_scenarios = []
        self.local_frame_rates = []

        self.current_workflow = None

        super().__init__()

    def run_workflow(self, path: Path):
        """
        Run workflow.

        Arguments
        ---------
        path : Path
            Path to the workflow file.

        Returns
        -------
        None

        """
        self.current_workflow = Workflow(
            path,
            self.global_frame_rate,
            )

        converted_scenarios = convert_scenarios(
            self.current_workflow.scenarios,
            self.output_directory)

        self.xosc_scenarios = [each.path for each in converted_scenarios]
        self.xosc_expects = [each.expect for each in converted_scenarios]
        self.local_frame_rates = [each.frame_rate for each in converted_scenarios]

        is_valid = XOSCValidator(False)

        for each in self.xosc_scenarios:
            if not is_valid(each):
                exit(1)

        self.run_all_scenarios()

    def spin(self):
        """Run scenario."""
        time.sleep(self.SLEEP_RATE)
        self.activate_node()
        start = time.time()

        while (time.time() - start) < self.global_timeout \
                if self.global_timeout is not None else True:

            if self.get_lifecycle_state() == 'inactive':
                self.get_logger().info(
                    "Simulator normally transitioned to the inactive state.")
                return
            else:
                time.sleep(self.SLEEP_RATE)

        self.get_logger().error("The simulation has timed out. Forcibly inactivate.")

        self.deactivate_node()

    def run_all_scenarios(
            self,
            # scenarios,
            ):
        """
        Run all scenarios.

        Arguments
        ---------
        scenarios : List[]
            The path to the workflow file.

        Returns
        -------
        None

        """
        for index, scenario in enumerate(self.xosc_scenarios):
            self.get_logger().info(
                "Run " + str(index + 1) + " of " + str(len(self.xosc_scenarios)))

            self.configure_node(
                expect=self.xosc_expects[index],
                output_directory=self.output_directory,
                real_time_factor=self.global_real_time_factor,
                scenario=scenario,
                frame_rate=self.local_frame_rates[index],
                )

            if self.get_lifecycle_state() == 'unconfigured':
                self.get_logger().error("Failed to configure interpreter")

            else:
                self.spin()
                self.cleanup_node()

        self.shutdown()

    # def __del__(self):
    #     pass


def main():
    parser = argparse.ArgumentParser(
        description="This provides batch processing of scenarios for Tier IV "
                    "Scenario Simulator.\nThis program doesn't work on its own "
                    "and he needs to be called via ROS2 launch as 'ros2 launch "
                    "scenario_test_runner scenario_test_runner.launch.py'. "
                    "When calling 'scenario_test_runner.launch.py', the "
                    "options described below must be given in the form "
                    "'option:=VALUE' instead of '--option VALUE'.")

    parser.add_argument(
        '--output-directory',
        default=Path("/tmp"),
        type=Path,
        help="Specify the output destination directory of the generated file "
             "including the result file.")

    parser.add_argument(
        '--global-frame-rate',
        default=30,
        type=float)

    parser.add_argument(
        '-x', '--global-real-time-factor',
        default=1.0,
        type=float,
        help="Specify the ratio of simulation time to real time. If you set a "
             "value greater than 1, the simulation will be faster than in "
             "reality, and if you set a value less than 1, the simulation will "
             "be slower than in reality.")

    parser.add_argument(
        '-t', '--global-timeout',
        default=30,
        type=float,
        help="Specify the simulation time limit. This time limit is independent "
             "of the simulation playback speed determined by the option "
             "real_time_factor. It also has nothing to do with OpenSCENARIO's "
             "SimulationTimeCondition.")

    parser.add_argument(
        '-s', '--scenario',
        default="/dev/null",
        type=Path,
        help="Specify a scenario file (.yaml or .xosc) you want to execute. If "
             "a workflow file is also specified by the'--workflow' option at "
             "the same time, this option takes precedence (that is, only one "
             "scenario passed to the --scenario option will be executed).")

    parser.add_argument(
        '-w', '--workflow',
        default="$(find-pkg-share scenario_test_runner)/workflow_example.yaml",
        type=Path,
        help='Specify <workflow>.yaml file you want to execute.')

    parser.add_argument('--ros-args', nargs='*')  # XXX DIRTY HACK
    parser.add_argument('-r', nargs='*')  # XXX DIRTY HACK

    args = parser.parse_args()

    test_runner = ScenarioTestRunner(
        global_frame_rate=args.global_frame_rate,
        global_real_time_factor=args.global_real_time_factor,
        global_timeout=args.global_timeout,
        output_directory=args.output_directory / 'scenario_test_runner',
        )

    if args.scenario != Path("/dev/null"):
        print(str(substitute_ros_package(args.scenario).resolve()))

    elif args.workflow != Path("/dev/null"):
        test_runner.run_workflow(substitute_ros_package(args.workflow).resolve())

    else:
        print("Option '--scenario' does not supprted.")


if __name__ == '__main__':
    """Entrypoint."""
    main()
