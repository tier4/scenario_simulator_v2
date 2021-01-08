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

# import os

from openscenario_utility.conversion import convert
from pathlib import Path
from scenario_test_runner.workflow import Scenario
from typing import List


class ConverterHandler():
    """class to handler scenario converter."""

    @staticmethod
    def convert_scenarios(
            scenarios: List[Scenario],
            launcher_path
            ):

        result = []

        for each in scenarios:

            if each.path.suffix == '.xosc':

                result.append(each)

            else:  # == '.yaml'

                directory = Path('/tmp/scenario_test_runner') / each.path.stem

                paths = convert(
                    each.path,
                    directory,
                    False
                    )

                for path in paths:

                    result.append(
                        Scenario(
                            path,
                            each.expect,
                            each.frame_rate
                            )
                        )

        return result


def main():
    pass


if __name__ == "__main__":
    main()
