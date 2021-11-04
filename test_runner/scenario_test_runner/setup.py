from glob import glob
import os

from setuptools import setup

package_name = "scenario_test_runner"

setup(
    name=package_name,
    version="0.5.5",
    packages=[package_name],
    data_files=[
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.*")),
        ("share/" + package_name + "/test/scenario", list(filter(lambda p : os.path.isfile(p), glob("test/scenario/*")))),
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", glob("resource/*")),
    ] + [
        ("share/" + package_name + "/" + catalog_dir, [fname for fname in glob(catalog_dir + "/*") if os.path.isfile(fname)])
        for catalog_dir in glob("test/scenario/catalog/*") if os.path.isdir(catalog_dir)
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taiki Tanaka, Tatsuya Yamasaki",
    maintainer_email="taiki.tanaka@tier4.jp, tatsuya.yamasaki@tier4.jp",
    description="scenario test runner package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "result_checker       = scenario_test_runner.result_checker:main",
            "scenario_test_runner = scenario_test_runner.scenario_test_runner:main",
        ]
    },
)
