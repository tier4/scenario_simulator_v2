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
import rclpy
import time

from openscenario_utility.validation import XOSCValidator
from pathlib import Path
from scenario_test_runner.converter_handler import ConverterHandler
from scenario_test_runner.workflow import Workflow, resolve_ros_package
from scenario_test_runner.lifecycle_controller import LifecycleController
from sys import exit


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
            log_directory: Path,  # DEPRECATED
            ):
        """
        Initialize the class ScenarioTestRunner.

        Arguments
        ---------
        global_timeout : int
            If the success or failure of the simulation is not determined even
            after the specified time (seconds) has passed, the simulation is
            forcibly terminated as a failure.

        log_directory : Path
            Deprecated.

        Returns
        -------
        None

        """
        self.global_frame_rate = global_frame_rate
        self.global_real_time_factor = global_real_time_factor
        self.global_timeout = global_timeout

        self.launcher_path = Path(__file__).resolve().parent.parent
        self.log_path = Path(resolve_ros_package(str(log_directory)))

        self.xosc_scenarios = []
        self.local_frame_rates = []

    def run_workflow(self, path: Path, no_validation):
        """
        Run workflow.

        Arguments
        ---------
        path : Path
            The path to the workflow file.

        Returns
        -------
        None

        """
        workflow = Workflow(path)

        self.yaml_scenarios = []

        expects = []
        local_frame_rates = []

        for scenario in workflow.scenarios:
            self.yaml_scenarios.append(scenario['path'])

            if 'expect' not in scenario:
                expects.append('success')
            else:
                expects.append(scenario['expect'])

            if 'frame-rate' not in scenario:
                local_frame_rates.append(self.global_frame_rate)
            else:
                local_frame_rates.append(float(scenario['frame-rate']))

        self.xosc_scenarios, self.xosc_expects, self.local_frame_rates \
            = ConverterHandler.convert_all_scenarios(
                self.yaml_scenarios, expects, local_frame_rates, self.launcher_path)

        if not no_validation.lower() in ["true", "t", "yes", "1"]:
            self.validate_all_scenarios()

        super().__init__()

        self.run_all_scenarios()

    def validate_all_scenarios(self):
        """Validate all scenarios."""
        is_valid = XOSCValidator(False)
        for scenario in self.xosc_scenarios:
            if not is_valid(scenario):
                exit()

    def monitor_state(self):
        start = time.time()

        while (time.time() - start) < self.global_timeout \
                if self.global_timeout is not None else True:

            current_state = self.get_lifecycle_state()

            if current_state == 'inactive':
                self.get_logger().info(
                    "Simulator normally transitioned to the inactive state.")
                return

            time.sleep(self.SLEEP_RATE)

        self.get_logger().error("The simulation has timed out. Forcibly inactivate.")

        self.deactivate_node()

    def run_scenario(self):
        """Run scenario."""
        time.sleep(self.SLEEP_RATE)
        self.activate_node()
        self.monitor_state()

    def run_all_scenarios(self):
        """
        Run all scenarios.

        Returns
        -------
        None

        """
        if not self.log_path.exists():
            self.log_path.mkdir(parents=True, exist_ok=True)

        for index, scenario in enumerate(self.xosc_scenarios):
            self.get_logger().info(
                "Run " + str(index + 1) + " of " + str(len(self.xosc_scenarios)))

            self.configure_node(
                expect=self.xosc_expects[index],
                log_path=self.log_path,
                real_time_factor=self.global_real_time_factor,
                scenario=scenario,
                frame_rate=self.local_frame_rates[index],
                )

            if self.get_lifecycle_state() == 'unconfigured':
                self.get_logger().error("Failed to configure interpreter")

            else:
                self.run_scenario()
                self.cleanup_node()

        self.shutdown()

    def __del__(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument(  # DEPRECATED
        '--log_directory',
        type=Path,
        default=Path('/tmp'),
        help='Specify log_directory you want to execute.')

    parser.add_argument(
        '--no_validation',
        default=False,
        help='Disable validation to generated scenarios.')

    parser.add_argument(
        '--global-frame-rate',
        type=float,
        default=30
        )

    parser.add_argument(
        '--global-real-time-factor',
        type=float,
        default=1.0,
        help="Specify the ratio of simulation time to real time. If you set a "
             "value greater than 1, the simulation will be faster than in "
             "reality, and if you set a value less than 1, the simulation will "
             "be slower than in reality.")

    parser.add_argument(
        '-s', '--scenario',
        help='Specify the scenario you want to execute.')

    parser.add_argument(
        '-t', '--global-timeout',
        default=30,
        help="Specify the simulation time limit. This time limit is independent "
             "of the simulation playback speed determined by the option "
             "real_time_factor. It also has nothing to do with OpenSCENARIO's "
             "SimulationTimeCondition.")

    parser.add_argument(
        '-w', '--workflow',
        type=str,
        help='Specify workflow you want to execute.')

    parser.add_argument('--ros-args', nargs='*')  # XXX DIRTY HACK
    parser.add_argument('-r', nargs='*')  # XXX DIRTY HACK

    args = parser.parse_args()

    ScenarioTestRunner(
        global_frame_rate=args.global_frame_rate,
        global_real_time_factor=args.global_real_time_factor,
        global_timeout=args.global_timeout,
        log_directory=args.log_directory,  # DEPRECATED
        ).run_workflow(
            Path(resolve_ros_package(args.workflow)).resolve(),
            args.no_validation)

    rclpy.shutdown()


if __name__ == '__main__':
    """Entrypoint."""
    main()
