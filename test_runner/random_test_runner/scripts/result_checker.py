#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025 TIER IV, Inc. All rights reserved.
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
#
# Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

import argparse
import sys
import xml.etree.ElementTree as ET

from pathlib import Path


class ResultChecker:
    """Check results of random test runner tests are all passed or not."""

    def __init__(self):
        pass

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
                    suite.attrib["name"], case.attrib["name"]
                )

                all_ok = all_ok and is_passed()

        sys.exit(0 if all_ok else 1)


def main():
    parser = argparse.ArgumentParser(
        description="Check if random test runner tests are all passed"
    )
    parser.add_argument("xml", help="path to result xml file")
    args = parser.parse_args()
    checker = ResultChecker()
    checker.check(args.xml)


if __name__ == "__main__":
    main()
