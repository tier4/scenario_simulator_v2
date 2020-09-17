#!/usr/bin/env python
# -*- coding: utf-8 -*-
from scenario_converter.scenario_logger import Logger
from collections import OrderedDict
import re
import itertools
import numpy as np


class ParameterSweeper:
    @staticmethod
    def linspace_generator(start, step, stop):
        num = 1
        if (step == 0):
            print(ZeroDivisionError)
        else:
            num = int((float((stop - start)) / float(step))) + 1
        return np.linspace(start, stop, num)

    @staticmethod
    def get_modifier_bindings(modifier_dict):
        list_bindings = []
        if (modifier_dict is None):
            return None
        for index, item in enumerate(modifier_dict):
            pair = item.items()
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
                lin = ParameterSweeper.linspace_generator(start, step,
                                                          stop).tolist()
                for index2, item2 in enumerate(lin):
                    lin_tuple_lists.append((val, item2))
                list_bindings.append(lin_tuple_lists)
        return list_bindings

    @staticmethod
    def test(modifier_dict, xosc_text):
        print(xosc_text)
        bind = ParameterSweeper.get_modifier_bindings(modifier_dict)
        for item in itertools.product(*bind):
            xosc_text = "CAR SPEED"
            Logger().print_process("parameter distribution at file: " +
                                   str(item))
            # print(item)
            for index2, item2 in enumerate(item):
                # print(item2)
                pattern = str(item2[0])
                repl = str(item2[1])
                print("replace " + str(item2[0]) + " --> " + str(item2[1]))
                xosc_text = re.sub(pattern, repl, xosc_text)
            Logger().print_success(xosc_text)
            return xosc_text
        return xosc_text


if __name__ == "__main__":
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
