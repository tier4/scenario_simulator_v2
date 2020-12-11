from setuptools import setup

package_name = 'scenario_test_utility'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name + '/OpenSCENARIO.xsd']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name + '/workflow_schema.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taiki Tanaka',
    maintainer_email='taiki.tanaka@tier4.jp',
    description='scenario test utility package for autoware',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert = scenario_test_utility.convert:main',
            'result_checker = scenario_test_utility.result_checker:main',
            'scenario_converter = scenario_test_utility.scenario_converter:main',
            'workflow_validator = scenario_test_utility.workflow_validator:main',
            'xosc_validator = scenario_test_utility.xosc_validator:main',
        ],
    },
)
