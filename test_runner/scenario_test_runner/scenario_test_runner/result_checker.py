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

import argparse
import sys
import xml.etree.ElementTree as ET
import os


class ResultChecker:
    """Class to check scenario testing result is good or not."""

    def __init__(self):
        pass

    def check(self, result):
        tree = ET.parse(result)
        testsuites = tree.getroot()
        for testsuite in testsuites:
            print("checking testsuite : " + testsuite.attrib['name'])
            index = 1
            for testcase in testsuite:
                xosc_path = testcase.attrib['name']
                print("[" + str(index) + "/" + str(testsuite.items()[1][1]) + "] checking result of " + xosc_path)
                for result in testcase:
                    if result.tag == "failure":
                        print("unexpected failure")
                        sys.exit(1)
                    if result.tag == "error":
                        print("unexpected error")
                        sys.exit(1)
                print("expected result")
                rosbag_dir = os.path.splitext(xosc_path)[0]
                print('checking log directory ' + rosbag_dir)
                db3_found = False
                metadata_found = False
                for filename in os.listdir(rosbag_dir):
                    if filename == "metadata.yaml":
                        metadata_found = True
                    elif os.path.splitext(filename)[1] == ".db3":
                        db3_found = True
                if db3_found and metadata_found:
                    print("rosbag file found")
                else:
                    print("rosnag not found")
                    sys.exit(1)
                index = index + 1
                print('')
        sys.exit(0)


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
    pass
