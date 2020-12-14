#!/usr/bin/env python
# -*- coding:utf-8 -*-

from glob import glob
from pathlib import Path
from setuptools import setup, find_packages


def read_file(path):
    try:
        with open(path) as file:
            return file.read()
    except IOError:
        return ""

def read_lines_as_list(filename):
    return open(filename).read().splitlines()

setup(
    name="openscenario_utility",
    version="0.0.0",  # read_version(),
    url="https://github.com/tier4/openscenario_utility",  # TODO
    license="Apache License 2.0",
    author="Tatsuya Yamasaki",
    author_email="tatsuya.yamasaki@tier4.jp",
    maintainer="Tatsuya Yamasaki",
    maintainer_email="tatsuya.yamasaki@tier4.jp",
    description="Command-line tools for ASAM OpenSCENARIO 1.0.0",
    long_description=read_file("README.md"),
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=read_lines_as_list('requirements.txt'),
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    include_package_data=True,
    zip_safe=False,
    entry_points={
        'console_scripts': [
            "convert-yaml2xosc = openscenario_utility.convert:main"
        ],
    },
    )
