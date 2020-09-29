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
import time

from scenario_common.logger import Logger


class LifeCycleController(Node):

    def __init__(self):
        super().__init__("scenario_runner_mock")
        self.state = None
        self.node_logger = self.get_logger()
        self.client_get_state = self.create_client(
            GetState, 'scenario_runner_mock/get_state')

        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_get_state.srv_name + ' service not available')

        self.client_change_state = self.create_client(
            ChangeState, "scenario_runner_mock/change_state")
        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.node_logger.warn(
                self.client_change_state.srv_name + ' service not available')
        self.activate_node()

    def activate_node(self):
        self.node_logger.info(self.get_lifecycle_state())
        self.set_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        self.node_logger.info(self.get_lifecycle_state())
        scenario = ["scenario1", "scenario2", "scenario3"]
        for index, val in enumerate(scenario):
            self.node_logger.info(val)
            self.run_scenario()
        self.set_lifecycle_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)

        self.node_logger.info(self.get_lifecycle_state())

        Logger.print_info("state is"+self.get_lifecycle_state())

    def run_scenario(self):
        # while(self.get_lifecycle_state())
        self.node_logger.info(self.get_lifecycle_state())
        self.set_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(4)
        self.node_logger.info(self.get_lifecycle_state())
        self.set_lifecycle_state(Transition.TRANSITION_DEACTIVATE)

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
        self.set_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    test_lifecycle = LifeCycleController()
    rclpy.spin(test_lifecycle)
    test_lifecycle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
