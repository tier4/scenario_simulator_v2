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


import json
import os
import sys
import tempfile
import time

# This script's name collides with the installed Python package.
# Temporarily remove sys.path[0] (the script's directory) so the
# package import resolves to the ament-installed package, not this file.
_sp0 = sys.path.pop(0)
from scenario_test_runner.report.report_generator import generate_report
from scenario_test_runner.report.rosbag_merger import merge_single_rosbag
sys.path.insert(0, _sp0)

import rclpy
from argparse import ArgumentParser
from glob import glob
from lifecycle_controller import LifecycleController
from openscenario_preprocessor_msgs.srv import CheckDerivativeRemained
from openscenario_preprocessor_msgs.srv import Derive
from openscenario_preprocessor_msgs.srv import Load
from openscenario_preprocessor_msgs.srv import SetParameter
from openscenario_utility.conversion import convert
from pathlib import Path
from rclpy.executors import ExternalShutdownException
from shutil import copytree, rmtree
from subprocess import run as subprocess_run
from sys import exit
from typing import List
from scenario import Scenario
from scenario import substitute_ros_package

# Workaround: diffusion planner model switching via symlink.
# The launcher hardcodes the model path below and cannot be modified.
# When this path becomes configurable, remove all model-switching code
# (search for MODEL_SYMLINK_PATH to find all related code).
MODEL_SYMLINK_PATH = Path("/opt/autoware/mlmodels/diffusion_planner_for_x2")


def convert_scenario_to_xosc(scenario: Scenario, output_directory: Path):

    result = []

    if scenario.path.suffix == ".xosc":
        result.append(scenario)

    else:  # == '.yaml' or == '.yml'
        for path in convert(scenario.path, output_directory / scenario.path.stem, False):
            result.append(Scenario(path, scenario.frame_rate))

    return result


def build_webautobag(final_dir, staging_dir, topics, logger):
    models = sorted(d for d in staging_dir.iterdir() if d.is_dir())
    if not models:
        return

    first_model = models[0]
    copytree(str(first_model), str(final_dir), dirs_exist_ok=True)

    for model_dir in models[1:]:
        for xosc in sorted(first_model.rglob("*.xosc")):
            rel = xosc.relative_to(first_model)
            target_bag = final_dir / rel.parent / xosc.stem
            run_bag = model_dir / rel.parent / xosc.stem
            if not target_bag.is_dir() or not run_bag.is_dir():
                continue
            merge_single_rosbag(target_bag, run_bag, model_dir.name, topics, logger)


class ScenarioTestRunner(LifecycleController):
    """
    Class to test scenarios.

    Attributes
    ----------
    SLEEP_RATE : int
        Time to sleep before next scenario.

    """

    SLEEP_RATE = 1

    def __init__(
        self,  # Arguments are alphabetically sorted
        global_frame_rate: float,
        global_real_time_factor: float,
        global_timeout: int,  # [sec]
        output_directory: Path,
        override_parameters: str,
        comparison_model_paths: list = None,
    ):
        """
        Initialize the class ScenarioTestRunner.

        Arguments
        ---------
        global_timeout : int
            If the success or failure of the simulation is not determined even
            after the specified time (seconds) has passed, the simulation is
            forcibly terminated as a failure.

        output_directory : Path
            Output destination directory of the generated file including the
            result file.

        Returns
        -------
        None

        """
        super().__init__(timeout=global_timeout)

        self.global_frame_rate = global_frame_rate
        self.global_real_time_factor = global_real_time_factor
        self.global_timeout = global_timeout

        self.output_directory = substitute_ros_package(output_directory)

        if self.output_directory.exists():
            glob_pattern = str(self.output_directory.resolve()) + "/*"
            remove_targets = glob(glob_pattern + "/*")
            for target in remove_targets:
                if os.path.isdir(target):
                    rmtree(target)
                else:
                    os.remove(target)
        self.output_directory.mkdir(parents=True, exist_ok=True)

        self.set_parameters_preprocessor_client = self.create_client(SetParameter,
                                                                     '/simulation/openscenario_preprocessor/set_parameter')

        while not self.set_parameters_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/set_parameter service not available, waiting again...')

        self.check_preprocessor_client = self.create_client(CheckDerivativeRemained,
                                                            '/simulation/openscenario_preprocessor/check')
        while not self.check_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/check service not available, waiting again...')

        self.derive_preprocessor_client = self.create_client(Derive,
                                                             '/simulation/openscenario_preprocessor/derive')
        while not self.derive_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/derive service not available, waiting again...')

        self.load_preprocessor_client = self.create_client(Load,
                                                           '/simulation/openscenario_preprocessor/load')
        while not self.load_preprocessor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/simulation/openscenario_preprocessor/load service not available, waiting again...')

        if len(override_parameters) > 0:
            for parameter_name, parameter_value in json.loads(override_parameters).items():
                request = SetParameter.Request()
                request.name = parameter_name
                request.value = str(parameter_value)
                future = self.set_parameters_preprocessor_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.result() is None:
                    self.print_debug('/simulation/openscenario_preprocessor/set_parameter: timeout')
                    exit(1)

        self.comparison_model_paths = comparison_model_paths or []
        self.model_backup_path = None  # MODEL_SYMLINK_PATH workaround

    def spin(self):
        """Run scenario."""
        time.sleep(self.SLEEP_RATE)
        while self.activate_node():
            start = time.time()
            while rclpy.ok():
                if self.get_lifecycle_state() == "inactive":
                    break
                elif ((time.time() - start) > self.global_timeout
                        if self.global_timeout is not None else False):
                    self.get_logger().error("The simulation has timed out. Forcibly inactivate.")
                    self.deactivate_node()
                    break
                else:
                    time.sleep(self.SLEEP_RATE)

    def execute_scenario(self, scenario: Scenario):
        # convert t4v2/xosc to xosc
        xosc_scenarios = convert_scenario_to_xosc(scenario, self.output_directory)

        # post to preprocessor
        for xosc_scenario in xosc_scenarios:
            if self.post_scenario_to_preprocessor(xosc_scenario):
                preprocessed_scenarios = []
                while self.derivative_remained_on_preprocessor():
                    future = self.derive_preprocessor_client.call_async(Derive.Request())
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

                    if future.result() is not None:
                        result = Scenario(future.result().path,
                                          future.result().frame_rate)
                        preprocessed_scenarios.append(result)
                    else:
                        self.print_debug(
                            'Exception while calling service /simulation/openscenario_preprocessor/derive: '
                            + str(future.exception()))
                        exit(1)

                self.run_preprocessed_scenarios(preprocessed_scenarios)
            else:
                exit(1)

    def run_scenario(self, scenario: Scenario):
        self.execute_scenario(scenario)
        self.shutdown()
        self.destroy_node()

    def run_preprocessed_scenarios(self, scenarios: List[Scenario]):
        """
        Run all given scenarios.

        Arguments
        ---------
        scenarios : List[Scenario]

        Returns
        -------
        None

        """
        try:
            length = len(scenarios)

            for index, each in enumerate(scenarios):

                self.get_logger().info(
                    "Run "
                    + str(each.path.name)
                    + " ("
                    + str(index + 1)
                    + " of "
                    + str(length)
                    + ")"
                )

                self.configure_node(
                    frame_rate=each.frame_rate,
                    output_directory=self.output_directory,
                    real_time_factor=self.global_real_time_factor,
                    scenario=each.path,
                )

                if self.get_lifecycle_state() == "unconfigured":
                    self.get_logger().error("Failed to configure interpreter")

                else:
                    self.spin()

                self.cleanup_node()

        except (KeyboardInterrupt, ExternalShutdownException):
            self.get_logger().warn("KeyboardInterrupt")
            self.destroy_node()
            rclpy.try_shutdown()
            exit(0)
        except OSError as e:
            self.get_logger().warn("OSError: {}".format(e))
        except Exception as e:
            self.get_logger().error("\x1b[1;31m{}\x1b[0m".format(e))

    # def __del__(self):
    #     pass

    def post_scenario_to_preprocessor(self, scenario: Scenario):

        request = Load.Request()
        request.path = str(scenario.path.absolute())
        request.frame_rate = scenario.frame_rate

        future = self.load_preprocessor_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return True
        else:
            self.print_debug('Exception while calling service /simulation/openscenario_preprocessor/load: '
                             + str(future.exception()))
            return False

    def derivative_remained_on_preprocessor(self):

        future = self.check_preprocessor_client.call_async(CheckDerivativeRemained.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result().derivative_remained
        else:
            self.print_debug('Exception while calling service /simulation/openscenario_preprocessor/check: '
                             + str(future.exception()))
            exit(1)

    def print_debug(self, message: str):
        self.get_logger().info(message)

    def validate_comparison_models(self):
        for p in self.comparison_model_paths:
            if not p.is_dir():
                raise RuntimeError(f"Comparison model path does not exist: {p}")
        self.get_logger().info(
            f"[Model Compare] {len(self.comparison_model_paths)} models: "
            f"{[p.name for p in self.comparison_model_paths]}"
        )

    # --- MODEL_SYMLINK_PATH workaround: begin ---

    def setup_model_symlink(self, model_path):
        symlink = MODEL_SYMLINK_PATH
        if self.model_backup_path is None and symlink.exists() and not symlink.is_symlink():
            self.model_backup_path = Path("/tmp") / (symlink.name + ".backup")
            subprocess_run(
                ["sudo", "mv", str(symlink), str(self.model_backup_path)], check=True
            )
            self.get_logger().info(
                f"[Model Switch] Backed up {symlink} -> {self.model_backup_path}"
            )
        if symlink.is_symlink():
            subprocess_run(["sudo", "rm", str(symlink)], check=True)
        subprocess_run(
            ["sudo", "ln", "-s", str(model_path), str(symlink)], check=True
        )
        self.get_logger().info(f"[Model Switch] {symlink} -> {model_path}")

    def teardown_model_symlink(self):
        symlink = MODEL_SYMLINK_PATH
        if symlink.is_symlink():
            subprocess_run(["sudo", "rm", str(symlink)], check=True)
        if self.model_backup_path and self.model_backup_path.exists():
            subprocess_run(
                ["sudo", "mv", str(self.model_backup_path), str(symlink)], check=True
            )
            self.get_logger().info(
                f"[Model Switch] Restored {symlink} from {self.model_backup_path}"
            )
            self.model_backup_path = None

    # --- MODEL_SYMLINK_PATH workaround: end ---

    def run_scenario_with_comparison(self, scenario, comparison_topics):
        self.validate_comparison_models()
        final_dir = self.output_directory

        staging_dir = Path(tempfile.mkdtemp(prefix="scenario_staging_"))
        try:
            for model_path in self.comparison_model_paths:
                self.get_logger().info(f"[Model Compare] Running model: {model_path.name}")
                self.setup_model_symlink(model_path)
                run_output = staging_dir / model_path.name
                run_output.mkdir(exist_ok=True)
                self.output_directory = run_output
                try:
                    self.execute_scenario(scenario)
                finally:
                    self.output_directory = final_dir
        finally:
            self.teardown_model_symlink()

        staging_link = final_dir / "staging"
        if staging_link.is_symlink() or staging_link.exists():
            staging_link.unlink()
        staging_link.symlink_to(staging_dir)

        build_webautobag(final_dir, staging_dir, comparison_topics, self.get_logger())

        report_paths = generate_report(final_dir)
        for p in report_paths:
            self.get_logger().info(f"[Model Compare] Report: {p}")

        self.shutdown()
        self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    parser = ArgumentParser()

    parser.add_argument("--output-directory", default=Path("/tmp"), type=Path)

    parser.add_argument("--override-parameters", default=None, type=str)

    parser.add_argument("--global-frame-rate", default=30, type=float)

    parser.add_argument("-x", "--global-real-time-factor", default=1.0, type=float)

    parser.add_argument("-t", "--global-timeout", default=180, type=float)

    parser.add_argument("-s", "--scenario", default="/dev/null", type=Path)

    parser.add_argument(
        "--comparison-model-paths", default="", type=str,
        help="Comma-separated paths to model directories. First is the host bag.",
    )

    parser.add_argument(
        "--comparison-topics", default="/planning/trajectory", type=str,
        help="Comma-separated topic names to merge from comparison model rosbags",
    )

    parser.add_argument("--ros-args", nargs="*")  # XXX DIRTY HACK
    parser.add_argument("-r", nargs="*")  # XXX DIRTY HACK

    args = parser.parse_args()

    test_runner = ScenarioTestRunner(
        global_frame_rate=args.global_frame_rate,
        global_real_time_factor=args.global_real_time_factor,
        global_timeout=args.global_timeout,
        output_directory=args.output_directory / "scenario_test_runner",
        override_parameters=args.override_parameters,
        comparison_model_paths=[
            Path(p.strip()) for p in args.comparison_model_paths.split(",") if p.strip()
        ],
    )

    if args.scenario != Path("/dev/null"):
        scenario = Scenario(
            substitute_ros_package(args.scenario).resolve(),
            args.global_frame_rate,
        )
        if args.comparison_model_paths:
            comparison_topics = [t.strip() for t in args.comparison_topics.split(",") if t.strip()]
            test_runner.run_scenario_with_comparison(scenario, comparison_topics)
        else:
            test_runner.run_scenario(scenario)
    else:
        print("No scenario is specified. Specify one.")


if __name__ == "__main__":
    """Entrypoint."""
    main()
