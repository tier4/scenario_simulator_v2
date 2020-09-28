#!/usr/bin/env python
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

from scenario_common.manager import Manager
import argparse
import yaml


class DatabaseHandler():

    @staticmethod
    def read_database(database_path):
        # database をopenしてデータを取り出す
        file = open(database_path, "r")
        database = yaml.safe_load(file)
        file.close()
        # あるか確認
        launch_path = Manager.path_checker(database["Launch"])
        log_path = database["Log"]
        Manager.mkdir(log_path)
        scenario_path = []
        for scenario in database["Scenario"]:
            # あるか確認
            Manager.path_checker(scenario)
            scenario_path.append(scenario)
        print("load scenario path from yaml")
        map_path = dict()
        map_database = database["Map"][0]
        print(type(map_database))
        for key, val in map_database.items():
            # あるか確認
            Manager.path_checker(val)
            map_path[key] = val
        print("")
        return launch_path, log_path, scenario_path, map_path


def main():
    parser = argparse.ArgumentParser(description='load arguements')
    # for testing start this script from scenario simulator.auto directory
    parser.add_argument(
        '--database', default="scenario_launcher/config/scenario_databse.yaml")
    args = parser.parse_args()
    launch_path, log_path, scenario_path, map_path = DatabaseHandler.read_database(
        args.database)


if __name__ == "__main__":
    main()
