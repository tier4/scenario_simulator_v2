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

from scenario_launcher.database_handler import DatabaseHandler
from scenario_launcher.monitoring_server import MonitoringServer
from scenario_launcher.result_reporter import Reporter
from scenario_common.manager import Manager
from subprocess import PIPE
from multiprocessing import Process
import argparse
import subprocess
import sys
import time
import xmlrpc.client


class Launcher:

    SERVER_URI = 'http://0.0.0.0:10000'
    SLEEP_RATE = 1
    PKILLER = "kill -9 `lsof -w -n -i tcp:10000| awk '{print $2}'|awk 'END{print}'`;"

    def __init__(self, timeout, database):
        self.timeout = timeout
        self.database_path = database
        self.roslaunch_parent = None
        self.monitoring_process = None
        self.scenario_counter = 0
        self.launch_path = ""
        self.log_path = ""
        self.scenario_list = dict()
        self.map_dict = dict()

    def main(self):
        subprocess.run(self.PKILLER, shell=True, stdout=PIPE, stderr=PIPE)
        self.client = xmlrpc.client.Server(self.SERVER_URI)
        self.launch_path, self.log_path, self.scenario_list, self.map_dict \
            = DatabaseHandler.read_database(self.database_path)
        self.run_all_scenarios()

    def wait_until_simulation_finished(self):
        start = time.time()
        Manager.print_process(
            "Set Maximum Simulation Time: " + str(self.timeout))
        while (time.time() - start) < self.timeout:
            print("    Running")
            time.sleep(self.SLEEP_RATE)

    def launch_runner(self):
        subprocess.run("ros2 launch scenario_launcher dummy_runner.launch.py",
                       shell=True, stdout=PIPE, stderr=PIPE)
        # subprocess.run("ros2 run scenario_launcher test_runner",
        #                shell=True, stdout=PIPE, stderr=PIPE)

    def run_server(self):
        monitoring = MonitoringServer()
        monitoring.run()
        return

    def run_scenario(self, scenario):
        self.scenario_counter = self.scenario_counter + 1
        time.sleep(self.SLEEP_RATE)
        self.launch_runner()
        self.wait_until_simulation_finished()
        Manager.print_process("Reached to Maximum Simulation Time")
        results = {}
        results['code'] = self.client.get_exit_status()
        results['simulation_time'] = self.client.get_simulation_time()
        results['traveled_distance'] = self.client.get_traveled_distance()
        Reporter.write_result(self.log_path, results, scenario)
        sys.exit(0)
        print("")

    def run_all_scenarios(self):
        self.monitoring_process = Process(target=self.run_server)
        self.monitoring_process.start()
        for index, scenario in enumerate(self.scenario_list):
            Manager.print_process("running scenario " + scenario)
            print(str(index), scenario)
            self.run_scenario(scenario)
        time.sleep(1)
        self.client.terminate_server()
        self.monitoring_process.terminate()

    def __del__(self):
        subprocess.run(self.PKILLER, shell=True, stdout=PIPE, stderr=PIPE)


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--timeout',
                        type=int,
                        default=10,
                        help='Specify simulation time limit in seconds. \
                  The default is 180 seconds.')
    # default path is scenario simulator.auto
    parser.add_argument('--database',
                        default='scenario_launcher/config/scenario_databse.yaml',
                        help='Specify the database path')

    parser.add_argument('--log',
                        default='screen',
                        help='Specify the type of log output.')

    parser.add_argument('--scenario',
                        help='Specify the scenario you want to execute.')

    args = parser.parse_args()
    launcher = Launcher(args.timeout, args.database)
    launcher.main()


if __name__ == '__main__':
    main()
