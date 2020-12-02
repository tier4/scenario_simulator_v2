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

from scenario_test_runner.converter_handler import ConverterHandler
from scenario_test_runner.database_handler import DatabaseHandler
from scenario_test_runner.lifecycle_controller import LifecycleController
from scenario_test_utility.logger import Logger
from scenario_test_utility.manager import Manager
from scenario_test_utility.xosc_validator import XoscValidator


class ScenarioTestRunner:
    """
    class to test scenarios.

    **Attributes**
    * SLEEP_RATE (`int`): time to sleep before next scenario
    """

    SLEEP_RATE = 1

    def __init__(self, timeout):
        self.timeout = timeout
        self.database_path = None
        self.lifecycle_controller = None
        self.launcher_path = ''
        self.log_path = ''
        self.scenarios = []
        self.xosc_scenarios = []
        self.xosc_step_time_ms = []

    def run_workflow(self, workflow, log_directory, use_validation):
        """
        Run workflow.

        **Args**

        * workflow,log_directory (`str`)

        **Returns**

        *None

        """
        self.launcher_path, self.log_path, self.scenarios \
            = DatabaseHandler.read_database(workflow, log_directory)
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
            = ConverterHandler.convert_all_scenarios(self.yaml_scenarios,
                                                     expects, step_times_ms,
                                                     self.launcher_path)
        if(use_validation):
            self.validate_all_scenarios()
        self.lifecycle_controller = LifecycleController()
        self.run_all_scenarios()

    def validate_all_scenarios(self):
        """Validate all scenarios."""
        validator = XoscValidator()
        Logger.print_separator('validating scenarios')
        for scenario in self.xosc_scenarios:
            validator.validate_xosc_file(scenario)

    def monitor_state(self):
        start = time.time()
        while (time.time() - start) < self.timeout:
            Logger.print_info('    Monitoring in Launcher')
            current_state = self.lifecycle_controller.get_lifecycle_state()
            Logger.print_info('    scenario runner state is ' + current_state)
            if(current_state == 'inactive'):
                Logger.print_process('    end of running')
                return
            time.sleep(self.SLEEP_RATE)
        Logger.print_warning('Reached to maximum simulation time')
        self.lifecycle_controller.deactivate_node()

    def run_scenario(self):
        """Run scenario."""
        Logger.print_process(
            'Set maximum simulation time: ' + str(self.timeout))
        time.sleep(self.SLEEP_RATE)
        self.lifecycle_controller.activate_node()
        self.monitor_state()
        print('')

    def run_all_scenarios(self):
        """
        Run all scenarios.

        **Args**

        * workflow,log_directory (`str`)

        **Returns**

        *None

        """
        Manager.mkdir(self.log_path)
        for index, scenario in enumerate(self.xosc_scenarios):
            print(str(index+1), scenario)
        for index, scenario in enumerate(self.xosc_scenarios):
            Logger.print_separator(
                'Test case ' + str(index+1) + ' of ' + str(len(self.xosc_scenarios)))
            self.lifecycle_controller.configure_node(
                scenario,
                self.xosc_expects[index],
                self.xosc_step_time_ms[index],
                self.log_path)
            if (self.lifecycle_controller.get_lifecycle_state() == 'unconfigured'):
                Logger.print_warning(
                    'Skip this scenario because of activation failure')
                continue
            self.run_scenario()
            self.lifecycle_controller.cleanup_node()
        self.lifecycle_controller.shutdown()

    def __del__(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--timeout',
                        type=int,
                        default=180,
                        help='Specify simulation time limit in seconds. \
                  The default is 180 seconds.')

    parser.add_argument('--log',
                        default='screen',
                        help='Specify the type of log output.')

    parser.add_argument('--scenario',
                        help='Specify the scenario you want to execute.')

    parser.add_argument('--workflow',
                        help='Specify workflow you want to execute')

    parser.add_argument('--log_directory',
                        help='Specify log_directory you want to execute',
                        default='/tmp')

    parser.add_argument('--use_validation',
                        help='use validation or not',
                        default=True)

    args = parser.parse_args()
    runner = ScenarioTestRunner(args.timeout)
    runner.run_workflow(args.workflow, args.log_directory, args.use_validation)


if __name__ == '__main__':
    """Entrypoint."""
    main()
