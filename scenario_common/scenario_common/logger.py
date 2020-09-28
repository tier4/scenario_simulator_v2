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

from logging import getLogger, StreamHandler, Formatter, FileHandler, DEBUG
import logging
import datetime
import pathlib


class Logger:

    @staticmethod
    def print_info(message):
        logger.info(message)

    @staticmethod
    def print_process(message):
        logger.process("\x1b[1;36m{0}\x1b[0m".format(message))

    @staticmethod
    def print_success(message):
        logger.success("\x1b[32m{0}\x1b[0m".format(message))

    @staticmethod
    def print_error(message):
        logger.success("\x1b[1;31m{0}\x1b[0m".format(message))

    @staticmethod
    def print_warning(message):
        logger.critical("\x1b[33m{0}\x1b[0m".format(message))

    @staticmethod
    def print_separator(message):
        message = "------- " + message + " -----"
        logger.process("\x1b[1;35m{0}\x1b[0m".format(message))

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
    def setup_logger(log_folder=None, modname=__name__):
        log_folder = '{0}-{1}.log'.format(
            pathlib.Path(log_folder).resolve(), datetime.date.today())
        logger = getLogger(modname)
        logger.setLevel(DEBUG)
        logging.PROCESS = 28
        logging.SUCCESS = 25
        logging.PROCESS = 22
        logging.addLevelName(logging.SUCCESS, 'SUCCESS')
        logging.addLevelName(logging.PROCESS, "PROCESS")
        setattr(logger, 'success', lambda message, *
                args: logger._log(logging.SUCCESS, message, args))
        setattr(logger, 'process', lambda message, *
                args: logger._log(logging.PROCESS, message, args))
        sh = StreamHandler()
        sh.setLevel(DEBUG)
        formatter = Formatter('%(message)s')
        sh.setFormatter(formatter)
        logger.addHandler(sh)
        fh = FileHandler(log_folder)
        fh.setLevel(DEBUG)
        fh_formatter = Formatter(
            '%(asctime)s - %(process)d - %(filename)s  - %(lineno)d - %(levelname)s - %(message)s')
        fh.setFormatter(fh_formatter)
        logger.addHandler(fh)
        print("detailed log folder: "+log_folder)
        return logger


logger = Logger.setup_logger(__file__)


def main():
    for i in range(11):
        Logger.print_progress_bar(i, 10)
    print("")
    Logger.print_process("Process")
    Logger.print_success("Success")
    Logger.print_warning("Warning")
    Logger.print_error("Failure")
    logger = Logger.setup_logger(__file__)
    logger.debug("debug info")
    logger.info("info")
    logger.critical("info")
    logger.warning("info")
    logger.error("error")


if __name__ == "__main__":
    main()
