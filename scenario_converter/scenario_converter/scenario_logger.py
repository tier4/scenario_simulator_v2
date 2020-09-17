#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os


class Logger():
    @staticmethod
    def print_timeout_message(timeout):
        print("    The simulation is forced to terminate after " +
              str(timeout) + " seconds.")

    @staticmethod
    def print_process(message):
        print("\x1b[36m" + message + "\x1b[0m")

    @staticmethod
    def print_success(message):
        print("\x1b[32m" + message + "\x1b[0m")

    @staticmethod
    def print_error(message):
        print("\x1b[1;31m" + message + "\x1b[0m")

    @staticmethod
    def print_exception(message):
        print("\x1b[33m" + message + "\x1b[33m")

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
    def fopen_with_check(path, mode="r+"):
        try:
            file = open(path, mode)
        except FileNotFoundError:
            Logger.print_exception("unable to fopen: " + path)
        return file

    @staticmethod
    def mkdir_with_check(path):
        print("Try mkdir: " + os.path.abspath(path), end="")
        try:
            os.makedirs(path)
        except FileExistsError:
            print("\n  But cancelled making directory", end="")
            if (os.path.exists(path)):
                print(" -> Because folder already exist")
            else:
                print(" -> Because of unkown failure")

    @staticmethod
    def check_read_path_is_valid(ppath, path):
        abspath = ""
        if (path[0] is "/"):
            abspath = path
        else:
            abspath = ppath + "/" + path

        if (os.path.exists(abspath)):
            print("exist: " + abspath)
            return abspath
        else:
            Logger.print_exception("not exist: " + abspath)

    @staticmethod
    def check_write_path_is_valid(ppath, path):
        abspath = ""
        if (path[0] is "/"):
            abspath = path
        else:
            abspath = ppath + "/" + path
        Logger.print_exception("not exist: " + abspath)
        return abspath


if __name__ == "__main__":
    for i in range(10):
        Logger.print_progress_bar(i, 10)
    Logger.print_timeout_message(180)
    Logger.print_success("Success")
    Logger.print_error("failure")
