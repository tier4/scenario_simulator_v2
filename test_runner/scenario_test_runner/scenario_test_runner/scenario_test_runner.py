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
from scenario_test_runner.database_handler import DatabaseHandler
from scenario_test_runner.lifecycle_controller import LifecycleController
from scenario_test_utility.manager import Manager
from sys import exit


class ScenarioTestRunner(LifecycleController):
    """
    class to test scenarios.

    **Attributes**
    * SLEEP_RATE (`int`): time to sleep before next scenario
    """

    SLEEP_RATE = 1

    def __init__(self, timeout):

        self.timeout = timeout
        self.database_path = None
        # self.lifecycle_controller = None
        self.launcher_path = Path(__file__).resolve().parent.parent
        self.log_path = ''
        self.scenarios = []
        self.xosc_scenarios = []
        self.xosc_step_time_ms = []

    def run_workflow(self, workflow, log_directory, no_validation):
        """
        Run workflow.

        **Args**

        * workflow
        * log_directory (`str`)

        **Returns**

        * None

        """
        self.log_path, self.scenarios = DatabaseHandler.read_database(workflow, log_directory)

        self.yaml_scenarios = []
        expects = []
        step_times_ms = []

        for scenario in self.scenarios:
            self.yaml_scenarios.append(scenario['path'])

            if 'expect' not in scenario:
                expects.append('success')
            else:
                expects.append(scenario['expect'])

            if 'step_time_ms' not in scenario:
                step_times_ms.append(2)
            else:
                step_times_ms.append(scenario['step_time_ms'])

        self.xosc_scenarios, self.xosc_expects, self.xosc_step_time_ms \
            = ConverterHandler.convert_all_scenarios(
                self.yaml_scenarios, expects, step_times_ms, self.launcher_path)

        if not no_validation.lower() in ["true", "t", "yes", "1"]:
            self.validate_all_scenarios()

        # self.lifecycle_controller = LifecycleController()
        super().__init__()

        self.run_all_scenarios()

    def validate_all_scenarios(self):
        """Validate all scenarios."""
        is_valid = XOSCValidator(False)
        # Logger.print_separator('validating scenarios')
        for scenario in self.xosc_scenarios:
            if not is_valid(scenario):
                exit()

    def monitor_state(self):
        start = time.time()

        while (time.time() - start) < self.timeout:
            # Logger.print_info('    Monitoring in Launcher')
            current_state = self.get_lifecycle_state()
            # Logger.print_info('    scenario runner state is ' + current_state)
            if current_state == 'inactive':
                # Logger.print_process('end of running')
                self.get_logger().info(
                    "Simulator normally transitioned to the inactive state.")
                return

            time.sleep(self.SLEEP_RATE)

        self.get_logger().error(
            "The simulation has timed out. Forcibly inactivate.")

        self.deactivate_node()

    def run_scenario(self):
        """Run scenario."""
        # Logger.print_process('Set maximum simulation time: ' + str(self.timeout))
        time.sleep(self.SLEEP_RATE)
        self.activate_node()
        self.monitor_state()
        print('')

    def run_all_scenarios(self):
        """
        Run all scenarios.

        **Args**

        * workflow
        * log_directory (`str`)

        **Returns**

        * None

        """
        Manager.mkdir(self.log_path)

        # for index, scenario in enumerate(self.xosc_scenarios):
        #     print(str(index+1), scenario)

        # success = failure = error = 0

        for index, scenario in enumerate(self.xosc_scenarios):
            self.get_logger().info(
                "Run " + str(index + 1) + " of " + str(len(self.xosc_scenarios)))

            self.configure_node(
                scenario,
                self.xosc_expects[index],
                self.xosc_step_time_ms[index],
                self.log_path)

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

    parser.add_argument(
        '--timeout',
        type=int,
        default=180,
        help='Specify simulation time limit in seconds.  The default is 180 seconds.',
        )

    parser.add_argument(
        '--log',
        default='screen',
        help='Specify the type of log output.',
        )

    parser.add_argument(
        '--scenario',
        help='Specify the scenario you want to execute.',
        )

    parser.add_argument(
        '--workflow',
        help='Specify workflow you want to execute.',
        )

    parser.add_argument(
        '--log_directory',
        default='/tmp',
        help='Specify log_directory you want to execute.',
        )

    parser.add_argument(
        '--no_validation',
        default=False,
        help='Disable validation to generated scenarios.',
        )

    args = parser.parse_args()

    ScenarioTestRunner(args.timeout).run_workflow(
        args.workflow, args.log_directory, args.no_validation)

    rclpy.shutdown()


if __name__ == '__main__':
    """Entrypoint."""
    main()
