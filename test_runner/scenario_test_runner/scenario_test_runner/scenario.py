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

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from re import sub


def substitute_ros_package(pathname: Path):
    def find_pkg_share(match):
        return get_package_share_directory(match.group(1))

    return Path(
        sub("\\$\\(find-pkg-share\\s+([^\\)]+)\\)", find_pkg_share, str(pathname))
    )

class Scenario:
    """
    Manages a scenario.

    Attributes
    ----------
    path: Path
        The path to a scenario.

    """

    def __init__(self, path: Path, frame_rate: float):

        self.path = substitute_ros_package(path).resolve()

        self.frame_rate = frame_rate

if __name__ == "__main__":
    """Entrypoint."""
    pass
