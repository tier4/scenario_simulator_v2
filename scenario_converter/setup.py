from setuptools import setup

package_name = 'scenario_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanaka-car',
    maintainer_email='ttatcoder@outlook.jp',
    description='scenario converter package for autoware.auto',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_coverter = scenario_converter.scenario_coverter:main'
        ],
    },
)
