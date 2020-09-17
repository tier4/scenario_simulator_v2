#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os.path import join, dirname
from scenario_logger import Logger
import json
import os
import roslaunch
import signal
import sys
import time
if sys.version_info.major == 3:
    from xmlrpc.server import SimpleXMLRPCServer
elif sys.version_info.major == 2:
    import SimpleXMLRPCServer as xmlrpc_server
else:
    Logger.print_exception("invalid python version")
    sys.exit(0)


class MonitoringServer:

    IS_DEBUG_MODE = True

    def __init__(self):
        signal.signal(signal.SIGINT, self.terminate_server)
        print("init monitoring server") if self.IS_DEBUG_MODE else None
        self.server = None
        self.is_simulation_running = False
        self.simulation_time = 0
        self.exit_status = 42
        self.traveled_distance = 0

    def __del__(self):
        self.server = None
        self.is_simulation_running = False

    def instanciate_server(self):
        if sys.version_info.major == 3:
            self.server = SimpleXMLRPCServer(("localhost", 10000),
                                             allow_none=True,
                                             logRequests=False)
        if sys.version_info.major == 2:
            self.server = xmlrpc_server.SimpleXMLRPCServer(
                ("localhost", 10000), allow_none=True, logRequests=False)
        self.server.register_introspection_functions()

    def terminate_server(self):
        if self.server is not None:
            self.server.shutdown()
        self.server = None
        time.sleep(1)

    def set_simulation_running(self, is_simulation_running):
        Logger.print_process("scenario_runner running:" +
                               str(is_simulation_running))
        self.is_simulation_running = is_simulation_running

    def set_exit_status(self, status):
        Logger.print_process("exit status:" + str(status))
        self.exit_status = status

    def update_traveled_distance(self, new_traveled_distance):
        self.traveled_distance = new_traveled_distance

    def update_simulation_time(self, new_simulation_time):
        self.simulation_time = new_simulation_time

    def get_exit_status(self):
        return self.exit_status

    def get_simulation_time(self):
        return self.simulation_time

    def get_traveled_distance(self):
        return self.traveled_distance

    def register_functions(self):
        Logger.print_process(
            "initialize server") if self.IS_DEBUG_MODE else None
        self.server.register_function(self.update_simulation_time)
        self.server.register_function(self.update_traveled_distance)
        self.server.register_function(self.set_simulation_running)
        self.server.register_function(self.set_exit_status)
        self.server.register_function(self.get_exit_status)
        self.server.register_function(self.get_simulation_time)
        self.server.register_function(self.get_traveled_distance)
        self.server.register_function(self.terminate_server)
        Logger.print_process(
            "registration finished") if self.IS_DEBUG_MODE else None

    def run(self):
        self.instanciate_server()
        self.register_functions()
        try:
            Logger.print_process("running server")
            self.server.serve_forever()
        except KeyboardInterrupt:
            Logger.print_exception("\nKeyboard interrupt received, exiting.")
            time.sleep(2)
            sys.exit(0)


if __name__ == "__main__":
    server = MonitoringServer()
    server.run()
