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

from scenario_common.logger import Logger


class Manager():

    @staticmethod
    def ask_continuation():
        Logger.print_process("continue ? \n [y/n]:")
        answer = input()
        if (answer is not "y"):
            Logger.print_info("abort creating files")
            sys.exit()

    @staticmethod
    def read_data(path, mode="r"):
        data = None
        try:
            with open(path, mode) as file:
                file_type = pathlib.Path(path).suffix
                if(file_type == ".yaml"):
                    data = yaml.safe_load(file)
                else:
                    data = file.read()
        except FileNotFoundError:
            Logger.print_warning("FileNotFoundError: " + str(path))
        return data

    @staticmethod
    def path_checker(path):
        path = str(path)
        is_path = os.path.exists(path)
        if(is_path):
            Logger.print_process("path: " + path + " exists")
        else:
            Logger.print_warning("path: " + path + " not exists")
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
            Logger.print_warning("unable to fopen: " + str(path))
        return data

    @staticmethod
    def mkdir(path):
        message = "Try mkdir: " + os.path.abspath(path)
        try:
            os.makedirs(path)
        except FileExistsError:
            message = message+"\n       But cancelled making directory"
            if (os.path.exists(path)):
                message = message+" -> Because folder already exist"
            else:
                message = message+" -> Because of unkown failure"
        Logger.print_info(message)


def main():
    pass


if __name__ == "__main__":
    main()
