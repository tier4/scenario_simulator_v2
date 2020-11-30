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

import re


class XmlRegex():
    """
    xml converter class

    **Attributes**
    * REPLACE_LOWER_PATTERN (`str`): regex for convert lower case pattern
    * ARRANGE_EMPTY_PATTERN (`str`): regex for convert empty value pattern
    * REPLACE_LOWER_PATTERN (`str`): regex for convert empty value pattern
    * ARRANGE_DOUBLE_PATTERN (`str`): regex for convert same value pattern
    * UPSIDE_REPEAT_PATTERN (`str`): regex for convert repeat same tag pattern
    * UPPER_TRUE_PATTERN (`str`): True -> "True"
    * UPPER_FALSE_PATTERN (`str`): False -> "False"
    """
    REPLACE_LOWER_PATTERN = re.compile(r"(^\w.*)")
    ARRANGE_EMPTY_PATTERN = re.compile(r"<([a-zA-Z]*)></\1>")
    ARRANGE_EMPTY_PATTERN2 = re.compile(r"<([a-zA-Z]*)(.*)></\1>")
    ARRANGE_DOUBLE_PATTERN = re.compile(r"(\s*)<(.[a-zA-Z]*)>\s*<\2>")

    UPSIDE_REPEAT_PATTERN = re.compile(r"<(\w*)>\s*<\1( .*[^/])>")
    DOWNSIDE_REPEAT_PATTERN = re.compile(r"</(\w+)>(\s*)</\1>")

    UPPER_TRUE_PATTERN = re.compile(r"(True|\"True\")")
    UPPER_FALSE_PATTERN = re.compile(r"(False|\"False\")")

    def __init__(self):
        print("init")

    @staticmethod
    def apply_regression(text):
        text = XmlRegex.replace_empty_case(text)
        text = XmlRegex.replace_repeat_case(text)
        text = XmlRegex.replace_double_case(text)
        text = XmlRegex.replace_true_false_pattern(text)
        return text

    @staticmethod
    def replace_empty_case(phase):
        pattern = XmlRegex.ARRANGE_EMPTY_PATTERN
        repl = r"<\1/>"
        phase = re.sub(pattern, repl, phase)
        pattern = XmlRegex.ARRANGE_EMPTY_PATTERN2
        repl = r"<\1\2/>"
        return re.sub(pattern, repl, phase)

    @staticmethod
    def replace_double_case(phase):
        pattern = XmlRegex.ARRANGE_DOUBLE_PATTERN
        repl = r"\1<\2>"
        return re.sub(pattern, repl, phase, re.X)

    @staticmethod
    def replace_repeat_case(phase):
        pattern = XmlRegex.UPSIDE_REPEAT_PATTERN
        repl = r"<\1\2>"
        phase = re.sub(pattern, repl, phase)

        pattern = re.compile(r"<(\w+)>((\s*<\1\s(.*)/>)+)\s*</\1>")
        phase = re.sub(pattern, r"\2", phase)

        pattern = XmlRegex.DOWNSIDE_REPEAT_PATTERN
        repl = r"</\1>"
        return re.sub(pattern, repl, phase)

    @staticmethod
    def replace_true_false_pattern(phase):
        pattern = XmlRegex.UPPER_TRUE_PATTERN
        repl = r'"true"'
        phase = re.sub(pattern, repl, phase)
        pattern = XmlRegex.UPPER_FALSE_PATTERN
        repl = r'"false"'
        return re.sub(pattern, repl, phase)


if __name__ == "__main__":
    """Entrypoint."""
    empty_case_test = XmlRegex.replace_empty_case("<tag></tag>")
    print("empty_case:", empty_case_test)
    double_case_test = XmlRegex.replace_double_case("</tag>\n\t</tag>")
    print("double:", double_case_test)
