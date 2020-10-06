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


import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from scenario_common.logger import Logger
from scenario_simulator_msgs.srv import LauncherMsg


class LifecycleController(Node):

    NODE_NAME = "scenario_runner_node"
    PARAMETER_NAME = "scenario"
    STATES = {}

    def __init__(self):
        rclpy.init(args=self.NODE_NAME)
        super().__init__(LifecycleController.NODE_NAME)
        self.state = None
        self.node_logger = self.get_logger()
        self.client_get_state = self.create_client(
            GetState, LifecycleController.NODE_NAME+"/get_state")
        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_get_state.srv_name + ' service not available')
        self.client_change_state = self.create_client(
            ChangeState, LifecycleController.NODE_NAME+"/change_state")
        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_change_state.srv_name + ' service not available')
        self.launcher_server = self.create_service(
            LauncherMsg, 'launcher_msg', self.send_scenario_service)
        self.send_scenario = ""

    def send_scenario_service(self, request, response):
        Logger.print_info("runner request: "+request)
        response.launcher_msg = self.send_scenario
        return response

    def configure_node(self):
        self.send_scenario = "RESPONSE!"
        self.node_logger.info(self.get_lifecycle_state())
        self.set_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        Logger.print_info("Configure -> scenario runner state is " +
                          self.get_lifecycle_state())
        self.node_logger.info(self.get_lifecycle_state())

    def activate_node(self, scenario):
        self.node_logger.info(scenario)
        self.send_scenario = scenario
        Logger.print_process("serv scenario: "+scenario)
        self.set_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        Logger.print_info("Activate -> scenario runner state is " +
                          self.get_lifecycle_state())
        self.node_logger.info(self.get_lifecycle_state())

    def deactivate_node(self):
        self.set_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        Logger.print_info("Deactivate -> scenario runner state is " +
                          self.get_lifecycle_state())
        self.node_logger.info(self.get_lifecycle_state())

    def cleanup_node(self):
        self.set_lifecycle_state(Transition.TRANSITION_CLEANUP)
        Logger.print_info("CleanUp -> scenario runner state is " +
                          self.get_lifecycle_state())
        self.node_logger.info(self.get_lifecycle_state())

    def set_lifecycle_state(self, transition_id):
        reqest = ChangeState.Request()
        reqest.transition.id = transition_id
        future = self.client_change_state.call_async(reqest)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().success

    def get_lifecycle_state(self):
        future = self.client_get_state.call_async(GetState.Request())
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().current_state.label

    def shutdown(self):
        self.set_lifecycle_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
        Logger.print_info(self.get_lifecycle_state())
        self.destroy_node()


def main(args=None):
    lifecycle_controller = LifecycleController()
    lifecycle_controller.configure_node()
    lifecycle_controller.activate_node("scenario1")
    lifecycle_controller.deactivate_node()
    lifecycle_controller.cleanup_node()
    lifecycle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
