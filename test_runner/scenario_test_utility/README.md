### Requirements

This package depends on these packages
- beautiful soap
- xmltodict
- argparse
- xmlplain

xmlplain package is now in PR for rosdep before merging to master use below instead
before that you need python3-pip
```
python3-pip
pip3 install -r requirements.txt
```

### How To Build
when building use these commands below
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --cmake-clean-cache --cmake-clean-first --packages-select scenario_test_utility --symlink-install
```


### Tier4 Format V2 -> Open Scenario Format

To convert open scenario, use these arguments

- input (required):
 tier4 format version2 yaml files in a specified directory
 refenrence directory is relative to current directory
- output (optinal):
 tier4 parameter distributed open scenario xosc files
 default is under current directory converted_xosc/ 
- log (optinal):
 path to converted log file default is under current directory log/converted.txt

execution using python or rosrun
```
python3 scenario_test_utility.py --input="input yaml file" --output="output directory" --log="converted log file" 

or 

ros2 run scenario_test_utility.py --input="path to input yaml file" --output="path to output directory" --log="path to converted log file"

```
