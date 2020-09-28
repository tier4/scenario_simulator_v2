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
import json
import pathlib


class Reporter():

    IS_DEBUG_MODE = True
    RESULT_FILE_NAME = "-result.json"

    # write out simulation result
    @staticmethod
    def write_result(log_dir, results, scenario_path):
        Logger.print_info("write result json")
        if results["code"] == 0:  # boost::exit_success
            results["message"] = "exit_success"
            Logger.print_success("-------> Success")
        elif results["code"] == 201:  # boost::exit_test_failure
            results["message"] = "exit_test_failure"
            Logger.print_error("-------> Failure")
        elif results["code"] == 1:  # boost::exit_failure
            results["message"] = "exit_failure"
            Logger.print_error("-------> TimeOver")
        elif results["code"] == 200:  # boost::exit_exception_failure
            results["message"] = "exit_exception_failure"
            Logger.print_error("-------> Invalid")
        else:
            results["message"] = "scenario_runner broken"
            Logger.print_error("-------> Broken")
        results["path"] = scenario_path
        result_file_name = pathlib.PurePosixPath(
            scenario_path).stem + Reporter.RESULT_FILE_NAME
        log_path = log_dir + result_file_name
        with open(log_path, 'w') as file:
            json.dump(results,
                      file,
                      indent=2,
                      ensure_ascii=False,
                      sort_keys=True,
                      separators=(',', ': '))
        Logger.print_process("write out result to: " + log_path)


if __name__ == "__main__":
    reporter = Reporter()
    dummy_results = {}
    dummy_results['code'] = 0
    dummy_results['simulation_time'] = 10
    dummy_results['traveled_distance'] = 20
    log_path = "./log"
    scenario_path = "scenario_launcher/open_scenario/test-1.xosc"
    reporter.write_result(log_path, dummy_results, scenario_path)
