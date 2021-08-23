#!/usr/bin/env python
# -*- coding:utf-8 -*-

from glob import glob
from pathlib import Path
from setuptools import setup, find_packages


package_name = "openscenario_utility"

setup(
    name=package_name,
    version="0.0.0",
    license="Apache License 2.0",
    author="Tatsuya Yamasaki",
    author_email="tatsuya.yamasaki@tier4.jp",
    maintainer="Tatsuya Yamasaki",
    maintainer_email="tatsuya.yamasaki@tier4.jp",
    description="Utility tools for ASAM OpenSCENARIO 1.0.0",
    packages=find_packages(),
    install_requires=[
        "PyYAML==5.4",
        "numpy==1.19.4",
        "setuptools",
        "xmlschema==1.3.1",
    ],
    include_package_data=True,
    package_data={"": ["resources/*.xsd"]},
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", glob("resource/*")),
    ],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "validate-xosc = openscenario_utility.validation:main",
            "yaml2xosc     = openscenario_utility.conversion:main",
            "validate-scenario = openscenario_utility.scenario_validation:validate_scenario",
            "validate-workflow = openscenario_utility.scenario_validation:validate_workflow"
        ]

    },
)
