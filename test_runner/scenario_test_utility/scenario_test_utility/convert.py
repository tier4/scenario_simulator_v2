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

from ament_index_python.packages import get_package_share_directory
from argparse import ArgumentParser
from pathlib import Path

import sys
import xmlschema
import yaml


def load(path):
    if path.exists():
        with path.open('r') as file:
            return yaml.safe_load(file)
    else:
        print('No such file or directory: ' + path)
        sys.exit()


def from_yaml(keyword, node):

    result = {}

    if isinstance(node, dict):
        #
        # ???: { ... }
        #
        for tag, value in node.items():

            if isinstance(value, list) and len(value) == 0:
                #
                # Tag: []
                #
                # => REMOVE
                #
                continue

            if str.islower(tag[0]):
                #
                # tag: { ... }
                #
                # => @tag: { ... }
                #
                result["@" + tag] = value
            else:
                #
                # Tag: { ... }
                #
                # => NO CHANGES
                #
                result[tag] = from_yaml(tag, value)

        return result

    elif isinstance(node, list):
        #
        # ???: [ ... ]
        #
        result = []

        for index, item in enumerate(node):
            result.append(from_yaml(keyword, item))

        return result

    elif isinstance(node, str):

        if str.islower(keyword[0]):
            result["@" + keyword] = node
        else:
            result[keyword] = node

        return result

    else:
        return None


def convert(input, output):

    path = Path(
        get_package_share_directory('scenario_test_utility')
        ).joinpath('../ament_index/resource_index/packages/OpenSCENARIO.xsd')

    schema = xmlschema.XMLSchema(str(path))

    try:
        C = schema.encode(
            from_yaml('OpenSCENARIO', load(input)),
            preserve_root=True,
            unordered=True,  # Reorder elements
            )

    except Exception as exception:
        print(exception)
        sys.exit()

    else:
        print(schema.is_valid(C))
        print(xmlschema.XMLResource(C).tostring())


def main():
    parser = ArgumentParser(description='Convert OpenSCENARIO.yaml into .xosc')

    parser.add_argument('--input', type=str, required=True)
    parser.add_argument('--output', type=str, default=Path.cwd())

    args = parser.parse_args()

    convert(Path(args.input).resolve(), Path(args.output).resolve())


if __name__ == "__main__":
    main()
