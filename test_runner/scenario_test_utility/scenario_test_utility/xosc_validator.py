#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Xosc validator."""

# Copyright 2020 TierIV.inc. All rights reserved.
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
import argparse
from ament_index_python.packages import get_package_share_directory
import xmlschema
from scenario_test_utility.logger import Logger


class XoscValidator():
    """
    xosc validation class
    """
    def __init__(self):
        share_directory_path = os.path.join(get_package_share_directory('scenario_test_utility'))
        xsd_path = os.path.join(share_directory_path, '../', 'ament_index', 'resource_index',
                                                      'packages', 'OpenSCENARIO.xsd')
        self.openscenario_schema = xmlschema.XMLSchema(xsd_path)

    def validate_xosc_file(self, xosc_path):
        if self.openscenario_schema.is_valid(xosc_path):
            Logger.print_info('scenario ' + xosc_path + ' is valid')
        else:
            Logger.print_error('scenario ' + xosc_path + ' is not valid')
            Logger.print_info(self.openscenario_schema.validate(xosc_path))


def main():
    parser = argparse.ArgumentParser(description='Validator for OpenSCENARIO .xosc file')
    parser.add_argument('xosc', help='path to .xosc file')
    args = parser.parse_args()
    validator = XoscValidator()
    validator.validate_xosc_file(args.xosc)


if __name__ == '__main__':
    """Entrypoint."""
    main()
