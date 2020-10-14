#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Autoware Foundation. All rights reserved.
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

import pathlib

from scenario_test_utility.logger import Logger
from scenario_test_utility.manager import Manager


class DatabaseHandler():

    @staticmethod
    def read_database(workflow_file="workflow_example.yaml"):
        Logger.print_separator("Reading workflow")
        launcher_package_path = pathlib.Path(__file__).resolve().parent.parent
        Logger.print_info("package path: " + str(launcher_package_path))
        database_path = launcher_package_path / "config" / workflow_file
        if (not Manager.check_existence(database_path)):
            launcher_package_path = pathlib.Path(
                Manager.get_file_dir(
                    __file__, workflow_file)).parent
            Manager.check_existence(database_path)
            database_path = launcher_package_path / "config" / workflow_file
        Logger.print_info("package path: " + str(launcher_package_path))
        database = Manager.read_data(database_path)
        log_path = str(launcher_package_path / database["Log"])
        scenarios = []
        for scenario in database["Scenario"]:
            scenario_path = str(launcher_package_path / scenario["path"])
            Manager.check_existence(scenario_path)
            scenario["path"] = scenario_path
            scenarios.append(scenario)
        return launcher_package_path, log_path, scenarios


def main():
    log_path, scenarios = DatabaseHandler.read_database()


if __name__ == "__main__":
    main()
