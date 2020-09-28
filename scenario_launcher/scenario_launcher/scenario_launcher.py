#!/usr/bin/env python
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

from scenario_common.logger import Logger
from scenario_common.manager import Manager
from scenario_launcher.database_handler import DatabaseHandler
from scenario_launcher.monitoring_server import MonitoringServer
from scenario_launcher.result_reporter import Reporter
import argparse
from multiprocessing import Process
import subprocess as sb
import time
import xmlrpc.client


class Launcher:

    SERVER_URI = 'http://0.0.0.0:10000'
    SLEEP_RATE = 1
    PKILLER = "kill -9 `lsof -w -n -i tcp:10000| awk '{print $2}'|awk 'END{print}'`;"

    def __init__(self, timeout):
        self.timeout = timeout
        self.database_path = None
        self.monitoring_process = None
        self.runner_process = None
        self.launch_path = ""
        self.log_path = ""
        self.scenario_list = dict()
        self.map_dict = dict()

    def main(self):
        sb.run(self.PKILLER, shell=True, stdout=sb.PIPE, stderr=sb.PIPE)
        self.client = xmlrpc.client.Server(self.SERVER_URI)
        self.log_path, self.scenario_list, self.map_dict \
            = DatabaseHandler.read_database()
        self.run_all_scenarios()

    @staticmethod
    def launch_runner():
        Logger.print_info("    start dummy runner")
        sb.run("ros2 launch scenario_launcher dummy_runner.launch.py",
               shell=True, stdout=sb.PIPE, stderr=sb.PIPE)

    @staticmethod
    def run_server():
        monitoring = MonitoringServer()
        monitoring.run()
        return

    def launcher_monitoring(self):
        start = time.time()
        while (time.time() - start) < self.timeout:
            Logger.print_info("    Monitoring in Launcher")
            if(not self.client.get_simulation_running()):
                Logger.print_error("    scenario runner not running")
                return
            else:
                Logger.print_info("    runner running")
            time.sleep(self.SLEEP_RATE)
        Logger.print_info("Reached to Maximum Simulation Time")

    def run_scenario(self, scenario):
        Logger.print_process(
            "Set Maximum Simulation Time: " + str(self.timeout))
        self.runner_process = Process(target=Launcher.launch_runner)
        self.runner_process.start()
        time.sleep(self.SLEEP_RATE)
        self.launcher_monitoring()
        results = {}
        results['code'] = self.client.get_exit_code()
        results['simulation_time'] = self.client.get_simulation_time()
        results['traveled_distance'] = self.client.get_traveled_distance()
        Reporter.write_result(self.log_path, results, scenario)
        print("")

    def run_all_scenarios(self):
        Logger.print_separator("scenario preprocess")
        Manager.mkdir(self.log_path)
        self.monitoring_process = Process(target=Launcher.run_server)
        self.monitoring_process.start()
        for index, scenario in enumerate(self.scenario_list):
            print(str(index+1), scenario)
        time.sleep(2)
        Manager.ask_continuation()
        for index, scenario in enumerate(self.scenario_list):
            Logger.print_separator("scenario launch " + str(index+1))
            Logger.print_process("running scenario " + scenario)
            self.run_scenario(scenario)
        self.monitoring_process.terminate()

    def __del__(self):
        sb.run(self.PKILLER, shell=True, stdout=sb.PIPE, stderr=sb.PIPE)


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--timeout',
                        type=int,
                        default=10,
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
