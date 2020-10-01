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

from scenario_common.logger import Logger
from scenario_common.manager import Manager
from scenario_launcher.database_handler import DatabaseHandler
from scenario_launcher.lifecycle_controller import LifecycleController


class Launcher:

    SERVER_URI = 'http://0.0.0.0:10000'
    SLEEP_RATE = 1
    PKILLER = "kill -9 `lsof -w -n -i tcp:10000| awk '{print $2}'|awk 'END{print}'`;"

    def __init__(self, timeout):
        self.timeout = timeout
        self.database_path = None
        self.lifecycle_controller = None
        self.launch_path = ""
        self.log_path = ""
        self.scenario_list = dict()
        self.map_dict = dict()

    def main(self):
        self.log_path, self.scenario_list, self.map_dict \
            = DatabaseHandler.read_database()
        self.lifecycle_controller = LifecycleController()
        self.lifecycle_controller.configure_node()
        self.run_all_scenarios()

    def launcher_monitoring(self):
        start = time.time()
        while (time.time() - start) < self.timeout:
            Logger.print_info("    Monitoring in Launcher")
            current_state = self.lifecycle_controller.get_lifecycle_state()
            Logger.print_info("    scenario runner state is " + current_state)
            if(current_state == "inactive"):
                Logger.print_error("    end of running")
                return
            time.sleep(self.SLEEP_RATE)
        Logger.print_warning("Reached to Maximum Simulation Time")
        self.lifecycle_controller.deactivate_node()

    def run_scenario(self, scenario):
        Logger.print_process(
            "Set Maximum Simulation Time: " + str(self.timeout))
        time.sleep(self.SLEEP_RATE)
        self.lifecycle_controller.activate_node(scenario)
        self.launcher_monitoring()
        print("")

    def run_all_scenarios(self):
        Logger.print_separator("scenario preprocess")
        Manager.mkdir(self.log_path)
        for index, scenario in enumerate(self.scenario_list):
            print(str(index+1), scenario)
        # Manager.ask_continuation()
        for index, scenario in enumerate(self.scenario_list):
            Logger.print_separator("scenario launch " + str(index+1))
            # Logger.print_process("running scenario " + scenario)
            self.run_scenario(scenario)
        self.lifecycle_controller.shutdown()

    def __del__(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--timeout',
                        type=int,
                        default=4,
                        help='Specify simulation time limit in seconds. \
                  The default is 180 seconds.')

    parser.add_argument('--log',
                        default='screen',
                        help='Specify the type of log output.')

    parser.add_argument('--scenario',
                        help='Specify the scenario you want to execute.')

    args = parser.parse_args()
    launcher = Launcher(args.timeout)
    launcher.main()


if __name__ == '__main__':
    main()
