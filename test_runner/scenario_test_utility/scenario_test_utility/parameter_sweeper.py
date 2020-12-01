#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TierIV.inc. All rights reserved.
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


import itertools
import numpy as np
import re
from collections import OrderedDict

from scenario_test_utility.logger import Logger


class ParameterSweeper:
    """Parameter Sweeper Class."""

    @staticmethod
    def iota(start, step, stop):
        """
        Calculate steps.

        **Args**

        * start, step, stop(`int`)

        **Returns**

        * `ndarray`

        """
        num = 1
        if (step == 0):
            print(ZeroDivisionError)
        else:
            num = int((float((stop - start)) / float(step))) + 1
        return np.linspace(start, stop, num)

    @staticmethod
    def make_modifier_bindings(modifier_dict):
        """
        Make modifier bindings.

        **Args**

        * modifier dict(`dictionary`)

        **Returns**

        * `list`

        """
        list_bindings = []
        if (modifier_dict is None):
            return None
        else:
            for index, item in enumerate(modifier_dict):
                pair = list(item.items())
                val = pair[0][1]
                if ("list" in modifier_dict[index]):
                    name_tuple_lists = []
                    lists = modifier_dict[index]["list"]
                    for index2, item2 in enumerate(lists):
                        name_tuple_lists.append((val, item2))
                    list_bindings.append(name_tuple_lists)
                else:
                    lin_tuple_lists = []
                    start = modifier_dict[index]["start"]
                    step = modifier_dict[index]["step"]
                    stop = modifier_dict[index]["stop"]
                    lin = ParameterSweeper.iota(start, step, stop).tolist()
                    for index2, item2 in enumerate(lin):
                        lin_tuple_lists.append((val, item2))
                    list_bindings.append(lin_tuple_lists)
            return list_bindings

    @staticmethod
    def test(modifier_dict, xosc_text):
        """Test method."""
        print(xosc_text)
        bind = ParameterSweeper.make_modifier_bindings(modifier_dict)
        for item in itertools.product(*bind):
            xosc_text = "CAR SPEED"
            Logger.print_process(
                "parameter distribution at file: " + str(item))
            for index2, item2 in enumerate(item):
                pattern = str(item2[0])
                repl = str(item2[1])
                print("replace " + str(item2[0]) + " --> " + str(item2[1]))
                xosc_text = re.sub(pattern, repl, xosc_text)
            Logger.print_success(xosc_text)
            return xosc_text
        return xosc_text


if __name__ == "__main__":
    """Entrypoint."""
    od = [
        OrderedDict([('name', 'CAR'), ('list', ["car0", "car1"])]),
        OrderedDict([('name', 'WALKER'), ('list', ["walker0", "walker1"])]),
        OrderedDict([('name', 'SPEED'), ('start', 1), ('step', 2),
                     ('stop', 5)]),
        OrderedDict([('name', 'POSITION_X'), ('start', 6), ('step', 3),
                     ('stop', 18)])
    ]
    xosc_text = "CAR SPEED"
    xosc_text = ParameterSweeper.test(od, xosc_text)
    print(xosc_text)
