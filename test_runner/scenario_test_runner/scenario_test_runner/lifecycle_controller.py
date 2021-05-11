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


import rcl_interfaces
import rclpy

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from pathlib import Path
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node


class LifecycleController(Node):
    """
    Class to control lifecycle.

    Attributes
    ----------
    NODE_NAME : str
        Node name to control lifecycle.

    """

    NODE_NAME = "openscenario_interpreter"

    def __init__(self):
        super().__init__(node_name="lifecycle_controller", namespace="simulation")

        self.client_get_state = self.create_client(
            GetState, LifecycleController.NODE_NAME + "/get_state"
        )

        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                self.client_get_state.srv_name + " service unavailable"
            )

        self.client_change_state = self.create_client(
            ChangeState, LifecycleController.NODE_NAME + "/change_state"
        )

        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                self.client_change_state.srv_name + " service unavailable"
            )

        self.current_scenario = ""
        self.client_set_parameters = self.create_client(
            rcl_interfaces.srv.SetParameters,
            LifecycleController.NODE_NAME + "/set_parameters",
        )

    def send_request_to_change_parameters(
        self,  # Arguments are alphabetically sorted
        expect,
        frame_rate: float,
        output_directory: Path,
        real_time_factor: float,
        scenario: Path,
    ):
        """Send request to change scenario interperter's parameters."""
        request = rcl_interfaces.srv.SetParameters.Request()

        request.parameters = [
            Parameter(
                name="intended_result",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=str(expect.name)
                ),
            ),
            Parameter(
                name="osc_path",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=str(scenario)
                ),
            ),
            Parameter(
                name="output_directory",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=str(output_directory),
                ),
            ),
            Parameter(
                name="local_real_time_factor",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=real_time_factor
                ),
            ),
            Parameter(
                name="local_frame_rate",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=frame_rate
                ),
            ),
        ]

        future = self.client_set_parameters.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future

    def configure_node(
        self,  # Arguments are alphabetically sorted
        expect,
        frame_rate: float,
        output_directory: Path,
        real_time_factor: float,
        scenario: str,
    ):
        """Configure node to chagnge state from unconfigure to inactive."""
        self.current_scenario = scenario

        while not self.send_request_to_change_parameters(
            expect=expect,
            frame_rate=frame_rate,
            output_directory=output_directory,
            real_time_factor=real_time_factor,
            scenario=self.current_scenario,
        ).done():
            self.get_logger().info("Failed to set parameters. Resending...")

        self.set_lifecycle_state(Transition.TRANSITION_CONFIGURE)

    def activate_node(self):
        """Activate node to chagnge state from inactive to activate."""
        self.set_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        # Logger.print_info(
        #     "Activate -> scenario runner state is " + self.get_lifecycle_state())
        # self.get_logger().info(self.get_lifecycle_state())

    def deactivate_node(self):
        """Dectivate node to chagnge state from active to inactive."""
        self.set_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        # Logger.print_info(
        #     "Deactivate -> scenario runner state is " + self.get_lifecycle_state())
        # self.get_logger().info(self.get_lifecycle_state())

    def cleanup_node(self):
        """Cleanup node to change state from inactive to unconfigure."""
        self.set_lifecycle_state(Transition.TRANSITION_CLEANUP)
        # Logger.print_info(
        #     "CleanUp -> scenario runner state is " + self.get_lifecycle_state())
        # self.get_logger().info(self.get_lifecycle_state())

    def set_lifecycle_state(self, transition_id):
        """
        Set lifecycle state.

        Arguments
        ---------
        transition_id : int

        Returns
        -------
        success : bool

        """
        reqest = ChangeState.Request()
        reqest.transition.id = transition_id
        future = self.client_change_state.call_async(reqest)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().success

    def get_lifecycle_state(self):
        """
        Get lifecycle state.

        Arguments
        ---------
        None

        Returns
        -------
        label : int

        """
        future = self.client_get_state.call_async(GetState.Request())
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().current_state.label

    def shutdown(self):
        """Shutdown lifecycle controller."""
        self.set_lifecycle_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
        # Logger.print_info(self.get_lifecycle_state())
        # self.destroy_node()


def main(args=None):
    lifecycle_controller = LifecycleController()
    lifecycle_controller.configure_node("scenario1")
    lifecycle_controller.activate_node()
    lifecycle_controller.deactivate_node()
    lifecycle_controller.cleanup_node()
    lifecycle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """Entrypoint."""
    main()
