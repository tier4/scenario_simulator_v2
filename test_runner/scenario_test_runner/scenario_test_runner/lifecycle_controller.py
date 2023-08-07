#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
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
    TIMEOUT_SEC_DEFAULT = 60.0

    def __init__(self, timeout=None):
        super().__init__(node_name="lifecycle_controller", namespace="simulation")

        self.timeout_sec = timeout if timeout is not None else self.TIMEOUT_SEC_DEFAULT

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

        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)

    def send_request_to_change_parameters(
        self,  # Arguments are alphabetically sorted
        frame_rate: float,
        output_directory: Path,
        real_time_factor: float,
        scenario: Path,
    ):
        """Send request to change scenario interpreter's parameters."""
        request = rcl_interfaces.srv.SetParameters.Request()

        request.parameters = [
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        return future

    def configure_node(
        self,  # Arguments are alphabetically sorted
        frame_rate: float,
        output_directory: Path,
        real_time_factor: float,
        scenario: str,
    ):
        """Configure node to change state from unconfigure to inactive."""
        self.current_scenario = scenario

        while not self.send_request_to_change_parameters(
            frame_rate=frame_rate,
            output_directory=output_directory,
            real_time_factor=real_time_factor,
            scenario=self.current_scenario,
        ).done():
            self.get_logger().info("Failed to set parameters. Resending...")

        state_expects = "unconfigured"
        if self.get_lifecycle_state() == state_expects:
            success = self.set_lifecycle_state(Transition.TRANSITION_CONFIGURE)
            return success
        else:
            self.get_logger().error(
                "\x1b[1;31mInterpreter is "
                + self.get_lifecycle_state()
                + " now, but "
                + state_expects
                + " expected.\x1b[0m"
            )
            return False

    def activate_node(self):
        """Activate node to change state from inactive to activate."""
        state_expects = "inactive"
        if self.get_lifecycle_state() == state_expects:
            return self.set_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        else:
            self.get_logger().error(
                "\x1b[1;31mInterpreter is "
                + self.get_lifecycle_state()
                + " now, but "
                + state_expects
                + " expected.\x1b[0m"
            )
            return False

    def deactivate_node(self):
        """Deactivate node to change state from active to inactive."""
        state_expects = "active"
        if self.get_lifecycle_state() == state_expects:
            return self.set_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        else:
            self.get_logger().error(
                "\x1b[1;31mInterpreter is "
                + self.get_lifecycle_state()
                + " now, but "
                + state_expects
                + " expected.\x1b[0m"
            )
            return False

    def cleanup_node(self):
        """Cleanup node to change state from inactive to unconfigure."""
        state_expects = "inactive"
        if self.get_lifecycle_state() == state_expects:
            return self.set_lifecycle_state(Transition.TRANSITION_CLEANUP)
        else:
            self.get_logger().error(
                "\x1b[1;31mInterpreter is "
                + self.get_lifecycle_state()
                + " now, but "
                + state_expects
                + " expected.\x1b[0m"
            )
            return False

    def shutdown(self):
        """Shutdown lifecycle controller."""
        self.get_logger().info("\x1b[33mShutdown interpreter.\x1b[0m")
        current_state = self.get_lifecycle_state()
        if current_state == "unconfigured":
            return self.set_lifecycle_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
        elif current_state == "inactive":
            return self.set_lifecycle_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)
        elif current_state == "active":
            return self.set_lifecycle_state(Transition.TRANSITION_ACTIVE_SHUTDOWN)
        else:
            self.get_logger().error(
                "\x1b[1;31mInterpreter is "
                + self.get_lifecycle_state()
                + " now, and it cannot transition to shuting down.\x1b[0m"
            )
            return False

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
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future, executor=self.executor, timeout_sec=self.timeout_sec)
        if future.result() is None:
            raise RuntimeError("Interpreter tried to set current lifecycle state, but failed.")
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
        rclpy.spin_until_future_complete(self, future, executor=self.executor, timeout_sec=self.timeout_sec)
        if future.result() is None:
            raise RuntimeError("Interpreter tried to get current lifecycle state, but failed.")
        return future.result().current_state.label


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
