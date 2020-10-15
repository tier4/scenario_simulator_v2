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
import re

from ament_index_python.packages import get_package_share_directory
from scenario_test_utility.logger import Logger
from scenario_test_utility.manager import Manager


class DatabaseHandler():

    @staticmethod
    def read_database(workflow_file):
        Logger.print_separator("Reading workflow")
        launcher_package_path = pathlib.Path(__file__).resolve().parent.parent
        workflow_path = ""
        Logger.print_separator(workflow_file)
        if pathlib.Path(workflow_file).is_absolute():
            workflow_path = workflow_file
        else:
            match_find_pkg_share = re.match('\$\(find-pkg-share\s+([^\)]+)\).*', workflow_file)
            if match_find_pkg_share != None:
                workflow_path = re.sub('\$\(find-pkg-share\s+([^\)]+)\)',
                get_package_share_directory(match_find_pkg_share.group(1)),
                workflow_file)
                Logger.print_separator(workflow_path)#workflow_path)
        database = Manager.read_data(workflow_path)
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
