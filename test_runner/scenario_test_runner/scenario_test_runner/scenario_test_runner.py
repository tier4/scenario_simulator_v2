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
import time
from pathlib import Path
from shutil import rmtree
from sys import exit
from typing import List

import rclpy
from openscenario_utility.conversion import convert
from openscenario_utility.validation import XOSCValidator
from scenario_test_runner.lifecycle_controller import LifecycleController
from scenario_test_runner.workflow import (
    Expect,
    Scenario,
    Workflow,
    substitute_ros_package,
)


def convert_scenarios(scenarios: List[Scenario], output_directory: Path):

    result = []

    for each in scenarios:

        if each.path.suffix == ".xosc":
            result.append(each)

        else:  # == '.yaml' or == '.yml'
            for path in convert(each.path, output_directory / each.path.stem, False):
                result.append(Scenario(path, each.expect, each.frame_rate))

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

        self.output_directory = substitute_ros_package(output_directory)

        if self.output_directory.exists():
            rmtree(self.output_directory)
        self.output_directory.mkdir(parents=True, exist_ok=True)

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
            # TODO self.global_real_time_factor,
        )

        converted_scenarios = convert_scenarios(
            self.current_workflow.scenarios, self.output_directory
        )

        is_valid = XOSCValidator(False)

        for each in converted_scenarios:
            if not is_valid(each.path):
                exit(1)

        self.run_scenarios(converted_scenarios)

    def spin(self):
        """Run scenario."""
        time.sleep(self.SLEEP_RATE)
        self.activate_node()
        start = time.time()

        while (
            (time.time() - start) < self.global_timeout
            if self.global_timeout is not None
            else True
        ):

            if self.get_lifecycle_state() == "inactive":
                self.get_logger().info(
                    "Simulator normally transitioned to the inactive state."
                )
                return
            else:
                time.sleep(self.SLEEP_RATE)

        self.get_logger().error("The simulation has timed out. Forcibly inactivate.")

        self.deactivate_node()

    def run_scenario(self, scenario: Scenario):
        converted_scenarios = convert_scenarios([scenario], self.output_directory)

        is_valid = XOSCValidator(False)

        for each in converted_scenarios:
            if not is_valid(each.path):
                exit(1)

        self.run_scenarios(converted_scenarios)

    def run_scenarios(self, scenarios: List[Scenario]):  # TODO RENAME
        """
        Run all given scenarios.

        Arguments
        ---------
        scenarios : List[Scenario]

        Returns
        -------
        None

        """
        length = len(scenarios)

        for index, each in enumerate(scenarios):

            self.get_logger().info(
                "Run "
                + str(each.path.name)
                + " ("
                + str(index + 1)
                + " of "
                + str(length)
                + ")"
            )

            self.configure_node(
                expect=each.expect,
                frame_rate=each.frame_rate,
                output_directory=self.output_directory,
                real_time_factor=self.global_real_time_factor,
                scenario=each.path,
            )

            if self.get_lifecycle_state() == "unconfigured":
                self.get_logger().error("Failed to configure interpreter")

            else:
                self.spin()
                self.cleanup_node()

        self.shutdown()

    # def __del__(self):
    #     pass


def main():
    rclpy.init(args="scenario_test_runner")

    parser = argparse.ArgumentParser()

    parser.add_argument("--output-directory", default=Path("/tmp"), type=Path)

    parser.add_argument("--global-frame-rate", default=30, type=float)

    parser.add_argument("-x", "--global-real-time-factor", default=1.0, type=float)

    parser.add_argument("-t", "--global-timeout", default=180, type=float)

    parser.add_argument("-s", "--scenario", default="/dev/null", type=Path)

    parser.add_argument(
        "-w",
        "--workflow",
        type=Path,
        default="$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml",
    )

    parser.add_argument("--ros-args", nargs="*")  # XXX DIRTY HACK
    parser.add_argument("-r", nargs="*")  # XXX DIRTY HACK

    args = parser.parse_args()

    test_runner = ScenarioTestRunner(
        global_frame_rate=args.global_frame_rate,
        global_real_time_factor=args.global_real_time_factor,
        global_timeout=args.global_timeout,
        output_directory=args.output_directory / "scenario_test_runner",
    )

    if args.scenario != Path("/dev/null"):
        print(str(substitute_ros_package(args.scenario).resolve()))

        test_runner.run_scenario(
            Scenario(
                substitute_ros_package(args.scenario).resolve(),
                Expect["success"],
                args.global_frame_rate,
            )
        )

    elif args.workflow != Path("/dev/null"):
        test_runner.run_workflow(substitute_ros_package(args.workflow).resolve())

    else:
        print("Neither the scenario nor the workflow is specified. Specify either one.")


if __name__ == "__main__":
    """Entrypoint."""
    main()
