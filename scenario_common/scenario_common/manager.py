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

import json
import os
import pathlib
import yaml


class Manager():

    @staticmethod
    def print_process(message):
        print("\x1b[36m", end="")
        print(message)
        print("\x1b[0m", end="")

    @staticmethod
    def print_success(message):
        print("\x1b[32m", end="")
        print(message)
        print("\x1b[0m", end="")

    @staticmethod
    def print_error(message):
        print("\x1b[1;31m", end="")
        print(message)
        print("\x1b[0m", end="")

    @staticmethod
    def print_exception(message):
        print("\x1b[33m", end="")
        print(message)
        print("\x1b[0m", end="")

    @staticmethod
    def print_separator(message):
        print("\x1b[35m", end="")
        print("------- " + message + " -----")
        print("\x1b[0m", end="")

    @staticmethod
    def print_progress_bar(i, max):
        progress_bar_size = 40
        current_progress = int(i * progress_bar_size / max)
        progress_bar = ('>' * current_progress) + \
            (' ' * (progress_bar_size - current_progress))
        print("\r" +
              "[{0}] {1} % \033[1C".format(progress_bar, int(i * 100. / max)),
              end='')
        return

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
            Manager.print_exception("FileNotFoundError: " + str(path))
        return data

    @staticmethod
    def path_checker(path):
        path = str(path)
        is_path = os.path.exists(path)
        if(is_path):
            Manager.print_process("path: " + path + " exists")
        else:
            Manager.print_exception("path: " + path + " not exists")
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
        except FileNotFoundError:
            Manager.print_exception("unable to fopen: " + str(path))
        return data

    @staticmethod
    def mkdir(path):
        print("Try mkdir: " + os.path.abspath(path), end="")
        try:
            os.makedirs(path)
        except FileExistsError:
            print("\n  But cancelled making directory", end="")
            if (os.path.exists(path)):
                print(" -> Because folder already exist")
            else:
                print(" -> Because of unkown failure")


if __name__ == "__main__":
    for i in range(11):
        Manager.print_progress_bar(i, 10)
    print("")
    Manager.print_process("Process")
    Manager.print_success("Success")
    Manager.print_exception("Exception")
    Manager.print_error("Failure")
