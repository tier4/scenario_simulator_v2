#!/usr/bin/env python3
"""Scenario post-process service node.

Advertises ``~/post_process`` (openscenario_interpreter_msgs/srv/PostProcess)
and runs the shell command carried in each request. Environment variables
describing the scenario result are exported to the subprocess so the command
can reference them.

This node is a generic shell executor: it has no ROS parameters, and the
command to invoke is entirely controlled by the caller via the service
request. This keeps the C++ interpreter decoupled from the execution
mechanism while letting launch files decide what to run.
"""

import os
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from openscenario_interpreter_msgs.srv import PostProcess


SIGINT_GRACE_SECONDS = 5
SIGKILL_GRACE_SECONDS = 2
DEFAULT_TIMEOUT_SECONDS = 60


class ScenarioPostProcessor(Node):
    def __init__(self) -> None:
        super().__init__("scenario_post_processor")
        self._service = self.create_service(
            PostProcess, "~/post_process", self._on_request
        )
        self.get_logger().info("scenario_post_processor ready")

    def _on_request(
        self, request: PostProcess.Request, response: PostProcess.Response
    ) -> PostProcess.Response:
        if not request.command:
            response.success = True
            response.message = "empty command; skipped"
            return response

        env = os.environ.copy()
        env.update(
            {
                "SCENARIO_OSC_PATH": request.osc_path,
                "SCENARIO_BAG_PATH": request.bag_path,
                "SCENARIO_OUTPUT_DIRECTORY": request.output_directory,
                "SCENARIO_RECORD_STORAGE_ID": request.record_storage_id,
                "SCENARIO_RESULT": request.result,
            }
        )

        timeout = (
            request.timeout_seconds
            if request.timeout_seconds > 0
            else DEFAULT_TIMEOUT_SECONDS
        )

        self.get_logger().info(
            f"running post-process (timeout={timeout}s): {request.command}"
        )

        try:
            proc = subprocess.Popen(
                ["/bin/sh", "-c", request.command],
                env=env,
                start_new_session=True,
            )
        except OSError as e:
            response.success = False
            response.message = f"failed to spawn post-process: {e}"
            self.get_logger().error(response.message)
            return response

        rc = self._wait_with_timeout(proc, timeout)

        if rc is None:
            response.success = False
            response.message = "post-process did not exit even after SIGKILL"
        elif rc == 0:
            response.success = True
            response.message = "post-process completed successfully"
        else:
            response.success = False
            response.message = f"post-process exited with status {rc}"

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response

    def _wait_with_timeout(
        self, proc: subprocess.Popen, timeout: int
    ) -> Optional[int]:
        sigint_after = max(1, timeout - SIGINT_GRACE_SECONDS)

        try:
            return proc.wait(timeout=sigint_after)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"post-process exceeded {sigint_after}s, sending SIGINT"
            )
            proc.send_signal(signal.SIGINT)

        try:
            return proc.wait(timeout=SIGKILL_GRACE_SECONDS)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "post-process did not exit after SIGINT, sending SIGKILL"
            )
            proc.kill()

        try:
            return proc.wait(timeout=SIGKILL_GRACE_SECONDS)
        except subprocess.TimeoutExpired:
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ScenarioPostProcessor()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
