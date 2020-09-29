import os
from glob import glob
from setuptools import setup

package_name = 'scenario_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanaka-car',
    maintainer_email='ttatcoder@outlook.jp',
    description='scenario launcher package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_launcher = scenario_launcher.scenario_launcher:main',
            'database_handler = scenario_launcher.database_handler:main',
            'result_reporter = scenario_launcher.result_reporter:main',
            'test_runner = scenario_launcher.test_runner.test_client:main',
            'lifecycle_controller = scenario_launcher.lifecycle_controller:main',
            'dummy_runner = scenario_launcher.test_runner.dummy_runner:main'
        ],
    },
)
