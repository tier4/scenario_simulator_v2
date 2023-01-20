#!/usr/bin/env python
# -*- coding:utf-8 -*-

from glob import glob
from pathlib import Path
from setuptools import setup, find_packages
from warnings import simplefilter
from pkg_resources import PkgResourcesDeprecationWarning
from setuptools import SetuptoolsDeprecationWarning

simplefilter("ignore", category=SetuptoolsDeprecationWarning)
simplefilter("ignore", category=PkgResourcesDeprecationWarning)


package_name = "openscenario_utility"

setup(
    name=package_name,
    version="0.5.5",
    license="Apache License 2.0",
    author="Tatsuya Yamasaki",
    author_email="tatsuya.yamasaki@tier4.jp",
    maintainer="Tatsuya Yamasaki",
    maintainer_email="tatsuya.yamasaki@tier4.jp",
    description="Utility tools for ASAM OpenSCENARIO 1.2.0",
    packages=find_packages(),
    install_requires=["PyYAML", "numpy", "setuptools<66.0.0", "xmlschema"],
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
            "validate-workflow = openscenario_utility.scenario_validation:validate_workflow",
        ]
    },
)
