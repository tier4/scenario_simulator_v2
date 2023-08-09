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

import argparse
import sys
import xml.etree.ElementTree as ET

from pathlib import Path


class ResultChecker:
    """Check results of scenario-test are all passed or not."""

    def __init__(self):
        pass

    def is_rosbag_directory(self, path):
        found_db3 = False
        found_metadata = False

        for each in path.iterdir():
            if each.name == "metadata.yaml":
                found_metadata = True
            elif each.suffix == ".db3":
                found_db3 = True

        return found_db3 and found_metadata

    def check(self, result):
        suites = ET.parse(result).getroot()

        all_ok = True

        for suite in suites:
            for case in suite:

                def is_passed():
                    if case.find("failure") is not None:
                        print("[FAILED] " + str(here) + ".xosc")
                        return False
                    elif case.find("error") is not None:
                        print("[ERROR!] " + str(here) + ".xosc")
                        return False
                    else:
                        print("[PASSED] " + str(here) + ".xosc")
                        return True

                here = Path(
                    suites.attrib["name"], suite.attrib["name"], case.attrib["name"]
                )

                this_is_ok = is_passed() and self.is_rosbag_directory(here)

                all_ok = all_ok and this_is_ok

        sys.exit(0 if all_ok else 1)


def main():
    parser = argparse.ArgumentParser(
        description="check scenario testing result is good or not"
    )
    parser.add_argument("xml", help="path to result xml file")
    args = parser.parse_args()
    checker = ResultChecker()
    checker.check(args.xml)


if __name__ == "__main__":
    """Entrypoint."""
    main()
    pass
