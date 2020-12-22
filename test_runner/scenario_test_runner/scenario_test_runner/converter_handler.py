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

from pathlib import Path
from openscenario_utility.conversion import convert


class ConverterHandler():
    """class to handler scenario converter."""

    @staticmethod
    def convert_all_scenarios(all_scenarios, expects, step_times_ms, launcher_path):
        """Convert all scenarios."""
        assert len(all_scenarios) == len(expects)
        sweeped_xosc_scenarios = []
        xosc_expects = []
        xosc_step_time_ms = []
        for index, scenario in enumerate(all_scenarios):
            if Path(scenario).suffix == ".xosc":
                sweeped_xosc_scenarios.append(scenario)
                xosc_expects.append(expects[index])
                xosc_step_time_ms.append(step_times_ms[index])
            else:
                output_dir = ConverterHandler.convert_scenario(index, scenario, launcher_path)
                xosc_scenarios = ConverterHandler.sweep_scenarios(output_dir)
                sweeped_xosc_scenarios.extend(xosc_scenarios)
                for each in xosc_scenarios:
                    xosc_expects.append(expects[index])
                    xosc_step_time_ms.append(step_times_ms[index])
        return sweeped_xosc_scenarios, xosc_expects, xosc_step_time_ms

    @staticmethod
    def convert_scenario(index, yaml_scenario_path: Path, launcher_path: Path):
        """Convert scenarios."""
        folder_name = Path(yaml_scenario_path).stem
        file_name = folder_name + "-" + str(index)
        output_dir = launcher_path.joinpath("test/scenario/converted", folder_name, file_name)
        # log_dir = str(Path(output_dir).parent) + "/converted.log"
        # ScenarioConverter.main(yaml_scenario_path, output_dir, log_dir)
        convert(Path(yaml_scenario_path), Path(output_dir))
        return output_dir

    @staticmethod
    def sweep_scenarios(converted_dirs):
        """Sweep all scenarios."""
        converted_scenarios = []
        for root, dirname, filenames in os.walk(converted_dirs):
            for filename in filenames:
                if (".xosc" in filename):
                    converted_scenarios.append(root+"/"+filename)
        return converted_scenarios


def main():
    pass


if __name__ == "__main__":
    """Entrypoint."""
    main()
