#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os


class FileHandler():

    @staticmethod
    def get_file_name(path):
        return os.path.splitext(os.path.basename(path))[0]

    @staticmethod
    def get_folder_name(path):
        return os.path.basename(os.path.dirname(path))

    @staticmethod
    def get_dir(path):
        return os.path.dirname(path)

    @staticmethod
    def get_dir_name(path):
        return os.path.dirname(path) + "/"


if __name__ == "__main__":
    dir_name = FileHandler.get_dir_name(__file__)
    dir_name = FileHandler().get_dir(__file__)
    file_name = FileHandler().get_folder_name(__file__)
