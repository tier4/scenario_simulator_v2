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

import re

from ament_index_python.packages import get_package_share_directory


def resolve_ros_package(sentence):
    match_find_pkg_share = re.match("\\$\\(find-pkg-share\\s+([^\\)]+)\\).*", sentence)
    if match_find_pkg_share is not None:
        sentence = re.sub("\\$\\(find-pkg-share\\s+([^\\)]+)\\)",
                          get_package_share_directory(match_find_pkg_share.group(1)),
                          sentence)
    return sentence
