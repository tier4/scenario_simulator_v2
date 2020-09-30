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

from scenario_common.logger import Logger
from xmlrpc.server import SimpleXMLRPCServer
import sys
import time


class MonitoringServer:

    IS_DEBUG_MODE = True

    def __init__(self):
        Logger.print_info(
            "init monitoring server") if self.IS_DEBUG_MODE else None
        self.server = None
        self.simulation_initialized = False
        self.simulation_running = True
        self.simulation_time = 0
        self.exit_status = 42
        self.traveled_distance = 0

    def __del__(self):
        self.server = None

    def instanciate_server(self):
        self.server = SimpleXMLRPCServer(
            ("localhost", 10000), allow_none=True, logRequests=False)

    def terminate_server(self):
        if self.server is not None:
            self.server.shutdown()
        self.server = None
        time.sleep(1)

    def set_simulation_initialized(self, simulation_initialized):
        Logger.print_info("    simulation initialized: " +
                          str(simulation_initialized))
        self.simulation_initialized = simulation_initialized

    def get_simulation_initialized(self): return self.simulation_initialized

    def set_simulation_running(self, simulation_running):
        Logger.print_info("    scenario_runner running: " +
                          str(simulation_running))
        self.simulation_running = simulation_running

    def get_simulation_running(self): return self.simulation_running

    def set_exit_code(self, status):
        Logger.print_info("    exit status: " + str(status)+"\n")
        self.exit_status = status

    def get_exit_code(self): return self.exit_status

    def update_traveled_distance(self, new_traveled_distance):
        Logger.print_info("    traveled distance: " +
                          str(new_traveled_distance))
        self.traveled_distance = new_traveled_distance

    def get_traveled_distance(self): return self.traveled_distance

    def update_simulation_time(self, new_simulation_time):
        Logger.print_info("    simulation time: " + str(new_simulation_time))
        self.simulation_time = new_simulation_time

    def get_simulation_time(self): return self.simulation_time

    def register_functions(self):
        self.server.register_function(self.set_simulation_running)
        self.server.register_function(self.get_simulation_running)
        self.server.register_function(self.set_exit_code)
        self.server.register_function(self.get_exit_code)
        self.server.register_function(self.update_simulation_time)
        self.server.register_function(self.get_simulation_time)
        self.server.register_function(self.update_traveled_distance)
        self.server.register_function(self.get_traveled_distance)
        self.server.register_function(self.terminate_server)

    def run(self):
        self.instanciate_server()
        self.register_functions()
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            time.sleep(2)
            sys.exit(0)


def main():
    server = MonitoringServer()
    server.run()


if __name__ == "__main__":
    main()
