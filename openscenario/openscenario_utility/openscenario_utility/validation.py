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

from pathlib import Path
from pkg_resources import resource_string
from sys import exit

import argparse
import xmlschema


class XOSCValidator:
    def __init__(self, verbose: bool = False):

        self.verbose = verbose

        self.schema = xmlschema.XMLSchema(
            resource_string(__name__, "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
        )

    def __call__(self, xosc: Path) -> bool:

        try:
            self.schema.validate(xmlschema.XMLResource(str(xosc)))

            if self.verbose:
                print("[OK] " + str(xosc))

            return True

        except Exception as exception:

            print("[NG] " + str(xosc))
            print()
            print("Error: " + str(exception))

            return False


def main():
    parser = argparse.ArgumentParser(
        description="Validate if the XOSC file complies with the ASAM OpenSCENARIO 1.2.0 standard"
    )

    parser.add_argument("paths", metavar="*.xosc", type=Path, nargs="+")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    validate = XOSCValidator(args.verbose)

    for each in args.paths:
        if not validate(each):
            exit()

    if args.verbose:
        print()

    print("All xosc files given are standard compliant.")


if __name__ == "__main__":
    """Entrypoint."""
    main()
