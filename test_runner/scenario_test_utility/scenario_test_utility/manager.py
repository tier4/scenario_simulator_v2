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

import json
import os
import pathlib
import sys
import yaml

from scenario_test_utility.logger import Logger


class Manager():

    @staticmethod
    def ask_continuation():
        Logger.print_process("Continue? \n [y/N]:")
        answer = input()
        if (answer is not "y"):
            Logger.print_info("Abort to make files.")
            sys.exit()

    @staticmethod
    def read_data(path, mode="r"):
        data = None
        try:
            with open(path, mode) as file:
                file_type = pathlib.Path(path).suffix
                if (file_type == ".yaml"):
                    data = yaml.safe_load(file)
                else:
                    data = file.read()
        except FileNotFoundError:
            Logger.print_warning("FileNotFoundError: " + str(path))
        return data

    @staticmethod
    def check_existence(path, verbose=False):
        path = str(path)
        is_path = os.path.exists(path)
        if verbose:
            if(is_path):
                Logger.print_process("Path " + path + " is exists.")
            else:
                Logger.print_warning("Path " + path + " is not exists.")
        return is_path

    @staticmethod
    def write_data(path, data, mode="w"):
        try:
            with open(path, mode) as file:
                file_type = pathlib.Path(path).suffix
                if(file_type == ".json"):
                    json.dump(data,
                              file,
                              indent=2,
                              ensure_ascii=False,
                              sort_keys=True,
                              separators=(',', ': '))
                else:
                    file.write(data)
        except IOError:
            Logger.print_warning("Failed to open " + str(path))
        return data

    @staticmethod
    def mkdir(path, verbose=False):
        if verbose:
            message = "Try mkdir: " + os.path.abspath(path)
        try:
            os.makedirs(path)
        except FileExistsError:
            if verbose:
                message = message + "\n       But cancelled making directory."
                if (os.path.exists(path)):
                    message = message + " -> Because folder already exist"
                else:
                    message = message + " -> Because of unkown failure"
                Logger.print_info(message)

    @staticmethod
    def get_file_dir(file_path, target_file_name):
        parent_dir = pathlib.Path(file_path).parent
        for i in range(8):
            parent_dir = pathlib.Path(parent_dir).parent
        for root, dirname, filename in os.walk(parent_dir):
            if (target_file_name in filename):
                return root


def main():
    file_dir = Manager.get_file_dir(__file__, "scenario_database.yaml")
    Logger.print_info(file_dir)


if __name__ == "__main__":
    main()
