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

import os
import pathlib
from scenario_common.logger import Logger
from scenario_converter.scenario_converter import ScenarioConverter


class ConverterHandler():

    @staticmethod
    def convert_all_scenarios(all_scenarios, launcher_path):
        sweeped_xosc_scenarios = []
        for index, scenario in enumerate(all_scenarios):
            if (pathlib.Path(scenario).suffix == ".xosc"):
                sweeped_xosc_scenarios.append(scenario)
            else:
                output_dir = ConverterHandler.convert_scenario(
                    index, scenario, launcher_path)
                xosc_scenarios = ConverterHandler.sweep_scenarios(output_dir)
                sweeped_xosc_scenarios.extend(xosc_scenarios)
        return sweeped_xosc_scenarios

    @staticmethod
    def convert_scenario(index, yaml_scenario_path, launcher_path):
        folder_name = pathlib.Path(yaml_scenario_path).stem
        file_name = folder_name + "-" + str(index)
        output_dir = str(launcher_path) + "/" + "converted_xosc" + \
            "/" + folder_name + "/" + file_name
        log_dir = str(pathlib.Path(output_dir).parent)+"/converted.log"
        ScenarioConverter.main(yaml_scenario_path, output_dir, log_dir)
        return output_dir

    @staticmethod
    def sweep_scenarios(converted_dirs):
        converted_scenarios = []
        for root, dirname, filenames in os.walk(converted_dirs):
            for filename in filenames:
                if (".xosc" in filename):
                    Logger.print_info
                    converted_scenarios.append(root+"/"+filename)
        return converted_scenarios


def main():
    yaml_scenario_path = [
        "/path/to/scenario1.yaml",
        "/path/to/scenario2.yaml"]
    launcher_path = "/path/to/scenario_launcher"
    ConverterHandler.convert_all_scenarios(yaml_scenario_path, launcher_path)


if __name__ == "__main__":
    main()
