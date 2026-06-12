from glob import glob

from setuptools import setup

package_name = "parallel_test_runner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kotaro Yoshimoto",
    maintainer_email="kotaro.yoshimoto@tier4.jp",
    description="Parallel local execution of Web.Auto scenarios (lockstep)",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "parallel_test_runner = parallel_test_runner.parallel_test_runner:main",
            "minimal_adapi_stub = parallel_test_runner.minimal_adapi_stub:main",
        ],
    },
)
