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


from collections import OrderedDict, defaultdict
from bs4 import BeautifulSoup
from pathlib import Path

import argparse
import copy
import itertools
import os
import re
import xmlplain
import xmltodict

from scenario_test_utility.parameter_sweeper import ParameterSweeper
from scenario_test_utility.xml_regex import XmlRegex
from scenario_test_utility.logger import Logger
from scenario_test_utility.manager import Manager


def generate_dict_from_yaml(path):
    """
    Generate dictionary from yaml data.

    **Args**

    * path (`str`): yaml path.

    **Returns**

    * `dict`: value or None.

    """
    if os.path.exists(path):
        with open(path, "r") as file:
            return xmlplain.obj_from_yaml(file)
    else:
        Logger.print_error("No such file or directory: " + path)


def find_modifiers(directory):
    """
    Find modifiers.

    **Args**

    * directory (`directory`): modifiers.

    **Returns**

    * `dict`: value or None.

    """
    for tag, value in directory.items():
        if tag == "ScenarioModifiers" and value is not None:
            for tag, value in value.items():
                if tag == "ScenarioModifier":
                    return value
    return None


def mark_attributes(keyword, syntax_tree):
    """
    Add attributes to tags.

    **Args**

    * keyword (`str`),syntax_tree (`dict`)

    **Returns**

    * `str`: keyword , `result`: dict.

    """
    result = OrderedDict()
    if isinstance(syntax_tree, OrderedDict) or isinstance(syntax_tree, dict):
        #
        # ???: { ... }
        #
        for tag, value in syntax_tree.items():
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
                new_tag, new_value = mark_attributes(tag, value)
                result[new_tag] = new_value
        return keyword, result

    if isinstance(syntax_tree, list):
        #
        # ???: [ ... ]
        #
        result = defaultdict(list)
        for index, item in enumerate(syntax_tree):
            new_key, new_value = mark_attributes(keyword, item)
            if str.islower(new_key[0]):
                #
                # tag: [ ... ]
                #
                # => @tag: [ ... ]
                #
                result["@" + new_key] = new_value
            else:
                #
                # Tag: [ ... ]
                #
                # => NO CHANGES
                #
                result[new_key].append(new_value)
        return keyword, result

    if isinstance(syntax_tree, str):
        if (str.islower(keyword[0])):
            result["@" + keyword] = syntax_tree
        else:
            result[keyword] = syntax_tree

    return keyword, result


class ScenarioConverter:
    """
    Tier IV scenario converter class.

    **Attributes**
    * OPENSCENARIO_TAG (`str`): tag for specify open scenario
    * IS_DEBUG_MODE (`bool`): change logger mode
    """

    OPENSCENARIO_TAG = "OpenSCENARIO"
    IS_DEBUG_MODE = False

    @staticmethod
    def main(yaml_path, xosc_dir, log_path):
        """
        Run scenario converter.

        **Args**

        * yaml_path,xosc_dir,log_path (`str`)

        **Returns**

        *None

        """
        Logger.print_separator("Scenario Preprocess")
        root_data = generate_dict_from_yaml(yaml_path)
        modifiers = find_modifiers(root_data)
        root_data.pop("ScenarioModifiers", None)
        xosc_dict = ScenarioConverter.extract_open_scenario(root_data)
        xosc_text = ScenarioConverter.convert_dict2xosc(xosc_dict, xosc_dir)
        Manager.mkdir(Path(log_path).parent)
        Manager.mkdir(xosc_dir)
        ScenarioConverter.distribute(
            modifiers, xosc_text, xosc_dir, yaml_path, log_path)
        print("")
        Logger.print_process("<= " + yaml_path)
        Logger.print_process("=> " + xosc_dir + "/")
        Logger.print_process("log: " + os.path.abspath(log_path))

    @staticmethod
    def extract_open_scenario(open_scenario):
        """
        Extract open scenario tree.

        **Args**

        * open_scenario (`dict`)

        **Returns**

        * `dict`: value

        """
        key, value = mark_attributes(
            ScenarioConverter.OPENSCENARIO_TAG, open_scenario)
        return value

    @staticmethod
    def convert_dict2xosc(xosc_dict, xosc_path):
        """
        Convert dictionary to xosc.

        **Args**

        * xosc_dict (`dict`),xosc_path (`str`)

        **Returns**

        * `str`: xosc_text

        """
        xosc_text = xmltodict.unparse(xosc_dict,
                                      encoding='utf-8',
                                      full_document=True,
                                      short_empty_elements=False,
                                      pretty=True)
        xosc_text = XmlRegex.apply_regression(xosc_text)
        return xosc_text

    @staticmethod
    def distribute(modifier_dict, xosc_text, xosc_dir, yaml_path, log_path):
        """
        Distribute parameters.

        **Args**

        * modifier_dict (`dict`), xosc_text, xosc_dir, yaml_path, log_path (`str`)

        """
        bindings = ParameterSweeper.make_modifier_bindings(modifier_dict)
        xosc_name = Path(yaml_path).stem
        id = 1

        def ret_path(xosc_dir, xosc_name, id):
            # .zfill(5)
            return xosc_dir + "/" + xosc_name + "-" + str(id) + ".xosc"

        num_files = 0
        if (bindings is not None):
            for item in itertools.product(*bindings):
                num_files = num_files + 1
                print("\r" + "\x1b[36m" +
                      "...caliculating the number of files" + "\x1b[0m",
                      end="")
        else:
            num_files = 1
        print("")
        Logger.print_process(str(num_files) + " files will be created")
        # Manager.ask_continuation()
        if (bindings is None):
            xosc_path = ret_path(xosc_dir, xosc_name, id)
            Logger.print_progress_bar(1, 1)
            print("\n")
            ScenarioConverter.write_converted_log(id, " None ", log_path,
                                                  xosc_path, yaml_path)
            ScenarioConverter.write_converted_xosc(xosc_text, xosc_path)
            return
        for item in itertools.product(*bindings):
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
                    str(Path(xosc_path).stem + ".xosc") +
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

    parser.add_argument(
        '--input', type=str, required=True,
        help='absolute path to input yaml file')

    parser.add_argument(
        '--output', type=str, default=os.getcwd() + "/" + "converted_xosc/open_scenarios",
        help='absolute path to output converterd xosc file')

    args = parser.parse_args()

    ScenarioConverter.main(
        args.input, args.output, Path(args.output).parent.joinpath("/converted.log"))


if __name__ == "__main__":
    """Entrypoint."""
    main()
