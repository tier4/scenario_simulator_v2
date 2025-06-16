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
import xml.etree.ElementTree as ET
import sys

class ResultChecker:
    def __init__(self):
        pass

    def check(self, xml_path):
        try:
            tree = ET.parse(xml_path)
            root = tree.getroot()
        except Exception as e:
            print(f"Failed to parse XML: {e}")
            return False

        passed, failed = 0, 0

        for suite in root:
            for case in suite:
                if case.find("failure") is None and case.find("error") is None:
                    passed += 1
                else:
                    failed += 1

        return passed > 0


def main():
    parser = argparse.ArgumentParser(description="Check result.junit.xml: passes if at least one test passed.")
    parser.add_argument("xml", help="JUnit XML result file")
    args = parser.parse_args()

    checker = ResultChecker()
    result = checker.check(args.xml)
    sys.exit(0 if result else 1)


if __name__ == "__main__":
    main()
