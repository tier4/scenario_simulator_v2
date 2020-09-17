#!/usr/bin/env python
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

import os


class FileHandler():

    @staticmethod
    def get_file_name(path):
        return os.path.splitext(os.path.basename(path))[0]

    @staticmethod
    def get_folder_name(path):
        return os.path.basename(os.path.dirname(path))

    @staticmethod
    def get_dir(path):
        return os.path.dirname(path)

    @staticmethod
    def get_dir_name(path):
        return os.path.dirname(path) + "/"


if __name__ == "__main__":
    dir_name = FileHandler.get_dir_name(__file__)
    dir_name = FileHandler().get_dir(__file__)
    file_name = FileHandler().get_folder_name(__file__)
