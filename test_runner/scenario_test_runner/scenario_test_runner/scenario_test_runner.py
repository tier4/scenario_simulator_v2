#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
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
import rclpy
import time

from argparse import ArgumentParser
from glob import glob
from lifecycle_controller import LifecycleController
from openscenario_preprocessor_msgs.srv import CheckDerivativeRemained
from openscenario_preprocessor_msgs.srv import Derive
from openscenario_preprocessor_msgs.srv import Load
from openscenario_utility.conversion import convert
from pathlib import Path
from rclpy.executors import ExternalShutdownException
from shutil import rmtree
from sys import exit
from typing import List
from workflow import Expect
from workflow import Scenario
from workflow import Workflow
from workflow import substitute_ros_package


def convert_scenarios_to_xosc(scenarios: List[Scenario], output_directory: Path):

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
        output_directory: Path
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
        super().__init__(timeout=global_timeout)

        self.global_frame_rate = global_frame_rate
        self.global_real_time_factor = global_real_time_factor
        self.global_timeout = global_timeout

        self.output_directory = substitute_ros_package(output_directory)

        if self.output_directory.exists():
            glob_pattern = str(self.output_directory.resolve()) + "/*"
            remove_targets = glob(glob_pattern + "/*")
            for target in remove_targets:
                if os.path.isdir(target):
                    rmtree(target)
                else:
                    os.remove(target)
        self.output_directory.mkdir(parents=True, exist_ok=True)

        self.current_workflow = None

        self.check_preprocessor_client = self.create_client(CheckDerivativeRemained,
                                                            '/simulation/openscenario_preprocessor/check')
        while not self.check_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/check service not available, waiting again...')

        self.derive_preprocessor_client = self.create_client(Derive,
                                                             '/simulation/openscenario_preprocessor/derive')
        while not self.derive_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/derive service not available, waiting again...')

        self.load_preprocessor_client = self.create_client(Load,
                                                           '/simulation/openscenario_preprocessor/load')
        while not self.load_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/load service not available, waiting again...')

        self.print_debug('connection established with preprocessor')

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

        converted_scenarios = convert_scenarios_to_xosc(
            self.current_workflow.scenarios, self.output_directory
        )

        self.run_scenarios(converted_scenarios)

    def spin(self):
        """Run scenario."""
        time.sleep(self.SLEEP_RATE)
        while self.activate_node():
            start = time.time()
            while rclpy.ok():
                if self.get_lifecycle_state() == "inactive":
                    break
                elif ((time.time() - start) > self.global_timeout
                        if self.global_timeout is not None else False):
                    self.get_logger().error("The simulation has timed out. Forcibly inactivate.")
                    self.deactivate_node()
                    break
                else:
                    time.sleep(self.SLEEP_RATE)

    def run_scenarios(self, scenarios: List[Scenario]):

        # convert t4v2/xosc to xosc
        xosc_scenarios = convert_scenarios_to_xosc(scenarios, self.output_directory)

        # post to preprocessor
        for xosc_scenario in xosc_scenarios:
            if self.post_scenario_to_preprocessor(xosc_scenario):
                preprocessed_scenarios = []
                while self.derivative_remained_on_preprocessor():
                    future = self.derive_preprocessor_client.call_async(Derive.Request())
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

                    if future.result() is not None:
                        result = Scenario(future.result().path,
                                          Expect(future.result().expect),
                                          future.result().frame_rate)
                        self.print_debug("derived : " + str(future.result().path))
                        preprocessed_scenarios.append(result)
                    else:
                        self.print_debug(
                            'Exception while calling service /simulation/openscenario_preprocessor/derive: '
                            + str(future.exception()))
                        exit(1)
                self.print_debug('finish derivation')

                for preprocessed_scenario in preprocessed_scenarios:
                    self.print_debug(str(preprocessed_scenario.path))

                self.run_preprocessed_scenarios(preprocessed_scenarios)
                self.print_debug('finish execution')
            else:
                exit(1)

        self.shutdown()
        self.destroy_node()

    def run_preprocessed_scenarios(self, scenarios: List[Scenario]):
        """
        Run all given scenarios.

        Arguments
        ---------
        scenarios : List[Scenario]

        Returns
        -------
        None

        """
        try:
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

        except (KeyboardInterrupt, ExternalShutdownException):
            self.get_logger().warn("KeyboardInterrupt")
            self.destroy_node()
            rclpy.try_shutdown()
            exit(0)
        except OSError as e:
            self.get_logger().warn("OSError: {}".format(e))
        except Exception as e:
            self.get_logger().error("\x1b[1;31m{}\x1b[0m".format(e))

    # def __del__(self):
    #     pass

    def post_scenario_to_preprocessor(self, scenario: Scenario):

        request = Load.Request()
        request.path = str(scenario.path.absolute())
        request.expect = scenario.expect
        request.frame_rate = scenario.frame_rate

        future = self.load_preprocessor_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.print_debug('Result of /simulation/openscenario_preprocessor/load: '
                             + str(future.result().message))
            return True
        else:
            self.print_debug('Exception while calling service /simulation/openscenario_preprocessor/load: '
                             + str(future.exception()))
            return False

    def derivative_remained_on_preprocessor(self):

        future = self.check_preprocessor_client.call_async(CheckDerivativeRemained.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            self.print_debug('Result of /simulation/openscenario_preprocessor/check: '
                             + str(future.result().derivative_remained))
            return future.result().derivative_remained
        else:
            self.print_debug('Exception while calling service /simulation/openscenario_preprocessor/check: '
                             + str(future.exception()))
            exit(1)

    def print_debug(self, message: str):
        self.get_logger().info(message)


def main(args=None):

    rclpy.init(args=args)

    parser = ArgumentParser()

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
        test_runner.run_scenarios(
            [Scenario(
                substitute_ros_package(args.scenario).resolve(),
                Expect["success"],
                args.global_frame_rate,
            )]
        )

    elif args.workflow != Path("/dev/null"):
        test_runner.run_workflow(substitute_ros_package(args.workflow).resolve())

    else:
        print("Neither the scenario nor the workflow is specified. Specify either one.")


if __name__ == "__main__":
    """Entrypoint."""
    main()
