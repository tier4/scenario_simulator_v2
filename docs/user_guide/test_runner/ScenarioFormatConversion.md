# Scenario Format Conversion
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

execution using ros2 command
```
ros2 run scenario_test_utility.py --input="path to input yaml file" --output="path to output directory" --log="path to converted log file"
```
