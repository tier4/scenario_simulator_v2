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

from scenario_converter.parameter_sweeper import ParameterSweeper
from scenario_converter.scenario_regressor import Regressor
from scenario_common.logger import Logger
from scenario_common.manager import Manager
from collections import OrderedDict, defaultdict
from bs4 import BeautifulSoup
import argparse
import copy
import itertools
import os
import pathlib
import re
import sys
import xmlplain
import xmltodict


class ScenarioConverter:

    OPEN_SCENARIO_TAG = "OpenSCENARIO"
    IS_DEBUG_MODE = False

    def __init__(self):
        self.xosc_path = ""

    def main(self, yaml_path, xosc_dir, log_path):
        ScenarioConverter.convert(yaml_path, xosc_dir, log_path)

    @staticmethod
    def convert(yaml_path, xosc_dir, log_path):
        Logger.print_separator("Scenario Preprocess")
        xosc_dict = None
        root_data = ScenarioConverter.convert_yaml2dict(yaml_path)
        length, modifier = ScenarioConverter.check_modifier_dict(root_data)
        if length == 1:
            xosc_dict = root_data
        else:
            xosc_dict = dict(list(root_data.items())[1:])
        xosc_dict = ScenarioConverter.extract_open_scenario(xosc_dict)
        xosc_text = ScenarioConverter.convert_dict2xosc(xosc_dict, xosc_dir)
        Manager.mkdir(pathlib.Path(log_path).parent)
        Manager.mkdir(xosc_dir)
        Logger.print_separator("Start Conversion")
        ScenarioConverter.apply_parameter_distribution(modifier, xosc_text,
                                                       xosc_dir, yaml_path,
                                                       log_path)
        print("")
        Logger.print_separator("Conversion Done")
        Logger.print_process("input: " + yaml_path)
        Logger.print_process("output: " + xosc_dir)
        Logger.print_process("log: " + os.path.abspath(log_path))

    @staticmethod
    def check_modifier_dict(root_data):
        scenario_modifiers = None
        scenario_modifier = None
        length = len(root_data)
        if "ScenarioModifiers" in root_data:
            scenario_modifiers = dict(root_data["ScenarioModifiers"])
            if "ScenarioModifier" in scenario_modifiers:
                scenario_modifier = scenario_modifiers["ScenarioModifier"]
        if scenario_modifier is None:
            Logger.print_info("No Scenario Modifiers Detected")
        return length, scenario_modifier

    @staticmethod
    def extract_open_scenario(open_scenario):
        key, val = ScenarioConverter.dfs_regressor(
            ScenarioConverter.OPEN_SCENARIO_TAG, open_scenario)
        xosc_dict = val
        return xosc_dict

    @staticmethod
    def dfs_regressor(keyword, value):
        print(value) if ScenarioConverter.IS_DEBUG_MODE else None
        o_dict = OrderedDict()
        d_dict = defaultdict(list)
        if isinstance(value, OrderedDict) or isinstance(value, dict):
            for key, val in value.items():
                if (str.islower(key[0])):
                    rpl_attr = Regressor.replace_lower_case(key)
                    o_dict[rpl_attr] = val
                else:
                    new_key, new_val = ScenarioConverter.dfs_regressor(
                        key, val)
                    o_dict[new_key] = new_val
            return keyword, o_dict
        if isinstance(value, list):
            for index, item in enumerate(value):
                new_key, new_val = ScenarioConverter.dfs_regressor(
                    keyword, item)
                if (str.islower(new_key[0])):
                    rpl_attr = Regressor.replace_lower_case(new_key)
                    d_dict[rpl_attr] = val
                else:
                    d_dict[new_key].append(new_val)
            return keyword, d_dict
        if isinstance(value, str):
            if (str.islower(keyword[0])):
                attr = Regressor.replace_lower_case(keyword)
                o_dict[attr] = value
            else:
                o_dict[keyword] = value
        return keyword, o_dict

    @staticmethod
    def convert_yaml2dict(yaml_scenario_path):
        Manager.path_checker(yaml_scenario_path)
        with open(yaml_scenario_path, "r") as file:
            root = xmlplain.obj_from_yaml(file)
        return root

    @staticmethod
    def convert_dict2xosc(xosc_dict, xosc_path):
        xosc_text = xmltodict.unparse(xosc_dict,
                                      encoding='utf-8',
                                      full_document=True,
                                      short_empty_elements=False,
                                      pretty=True)
        xosc_text = Regressor.apply_regression(xosc_text)
        return xosc_text

    @staticmethod
    def apply_parameter_distribution(modifier_dict, xosc_text, xosc_dir,
                                     yaml_path, log_path):
        bind = ParameterSweeper.get_modifier_bindings(modifier_dict)
        xosc_name = pathlib.Path(yaml_path).stem
        id = 1

        def ret_path(xosc_dir, xosc_name, id):
            return xosc_dir + "/" + xosc_name + "-" + str(
                id) + ".xosc"  # .zfill(5)

        num_files = 0
        if (bind is not None):
            for item in itertools.product(*bind):
                num_files = num_files + 1
                print("\r" + "\x1b[36m" +
                      "...caliculating the number of files" + "\x1b[0m",
                      end="")
        else:
            num_files = 1
        print("")
        Logger.print_process(str(num_files) + " files will be created")
        Manager.ask_continuation()
        if (bind is None):
            xosc_path = ret_path(xosc_dir, xosc_name, id)
            Logger.print_progress_bar(1, 1)
            print("\n")
            ScenarioConverter.write_converted_log(id, " None ", log_path,
                                                  xosc_path, yaml_path)
            ScenarioConverter.write_converted_xosc(xosc_text, xosc_path)
            return
        for item in itertools.product(*bind):
            xosc_path = ret_path(xosc_dir, xosc_name, id)
            converted_xosc_text = copy.deepcopy(xosc_text)
            Logger.print_progress_bar(id, num_files)
            ScenarioConverter.write_converted_log(id, item, log_path,
                                                  xosc_path, yaml_path)
            for index2, item2 in enumerate(item):
                pattern = str(item2[0])
                repl = str(item2[1])
                converted_xosc_text = re.sub(pattern, repl,
                                             converted_xosc_text)
            ScenarioConverter.write_converted_xosc(converted_xosc_text,
                                                   xosc_path)
            id = id + 1
        return

    @staticmethod
    def write_converted_log(id, item, log_path, xosc_path, yaml_path):
        log_text = (" file name: " +
                    str(pathlib.Path(xosc_path).stem + ".xosc") +
                    " parameter distribution case " + str(id) + "\033[1A")
        Logger.print_process(log_text)
        with open(log_path, 'a') as f:
            f.write(log_text)

    @staticmethod
    def write_converted_xosc(xosc_text, xosc_path):
        converted_xosc_text = copy.deepcopy(xosc_text)
        xosc_text = converted_xosc_text.encode("utf-8")
        xosc_text = BeautifulSoup(xosc_text, 'xml')
        xosc_text = xosc_text.prettify()
        Manager.write_data(xosc_path, xosc_text.encode("utf-8"), "wb")
        return


def main():
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--input',
                        type=str,
                        default="test.yaml",
                        help='input yaml file',
                        required=True)

    parser.add_argument('--output',
                        type=str,
                        default="converted_xosc",
                        help='output of converterd xosc')

    parser.add_argument('--log',
                        type=str,
                        default='log/converted.txt',
                        help='output of converterd log')

    args = parser.parse_args()
    input_dir = os.getcwd() + "/" + args.input
    output_dir = os.getcwd() + "/" + "converted_xosc"

    scenarioConverter = ScenarioConverter()
    scenarioConverter.main(input_dir, output_dir, args.log)


if __name__ == "__main__":
    main()
