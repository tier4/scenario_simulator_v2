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
import rcl_interfaces
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from scenario_test_utility.logger import Logger


class LifecycleController(Node):

    NODE_NAME = "openscenario_interpreter_node"
    PARAMETER_XOSC_PATH = "osc_path"
    PARAMETER_EXPECT = "expect"
    STATES = {}

    def __init__(self):
        rclpy.init(args=self.NODE_NAME)
        super().__init__(LifecycleController.NODE_NAME)

        self.state = None
        self.node_logger = self.get_logger()

        self.client_get_state = self.create_client(
            GetState, LifecycleController.NODE_NAME + "/get_state")

        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_get_state.srv_name + ' service not available')

        self.client_change_state = self.create_client(
            ChangeState, LifecycleController.NODE_NAME + "/change_state")

        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_change_state.srv_name + ' service not available')

        self.current_scenario = ""
        self.client_set_parameters = self.create_client(
            rcl_interfaces.srv.SetParameters,
            LifecycleController.NODE_NAME + '/set_parameters')

    def send_request_to_change_parameters(self, scenario, expect, step_time_ms, log_path):
        request = rcl_interfaces.srv.SetParameters.Request()
        request.parameters = [
            rcl_interfaces.msg.Parameter(
                name='step_time_ms',
                value=rcl_interfaces.msg.ParameterValue(
                    type=rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER,
                    integer_value=step_time_ms
                )
            ),
            rcl_interfaces.msg.Parameter(
                name=LifecycleController.PARAMETER_XOSC_PATH,
                value=rcl_interfaces.msg.ParameterValue(
                    type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                    string_value=scenario
                )
            ),
            rcl_interfaces.msg.Parameter(
                name=LifecycleController.PARAMETER_EXPECT,
                value=rcl_interfaces.msg.ParameterValue(
                    type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                    string_value=expect
                )
            ),
            rcl_interfaces.msg.Parameter(
                name="log_path",
                value=rcl_interfaces.msg.ParameterValue(
                    type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                    string_value=log_path
                )
            )
        ]
        future = self.client_set_parameters.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future

    def configure_node(self, scenario, expect, step_time_ms, log_path):
        self.node_logger.info(self.get_lifecycle_state())

        self.current_scenario = scenario
        Logger.print_process(
            "Set value '" +
            self.current_scenario +
            "' to " +
            LifecycleController.NODE_NAME +
            "'s parameter 'scenario'")

        while not self.send_request_to_change_parameters(
                self.current_scenario, expect, step_time_ms, log_path).done():
            Logger.print_info('Failed to set parameters. Resending...')

        self.set_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        Logger.print_info("Configure -> scenario runner state is " +
                          self.get_lifecycle_state())

        self.node_logger.info(self.get_lifecycle_state())

    def activate_node(self):
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
    lifecycle_controller.configure_node("scenario1")
    lifecycle_controller.activate_node()
    lifecycle_controller.deactivate_node()
    lifecycle_controller.cleanup_node()
    lifecycle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
