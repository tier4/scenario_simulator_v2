# Scenario Format Conversion
### Tier4 Format V2 -> OpenSCENARIO Format

To convert OpenSCENARIO, use these arguments

| Input | Required  | Description |
| ----- | --------- | ------------ |
| input | yes       | Tier4 format version2 yaml files in a specified directory, reference directory is relative to the current directory. |
| output | no       | Tier4 parameter distributed OpenSCENARIO .xosc files are generated in a specified directory. Default is under the current directory converted_xosc. |

You can execute scenario conversion by using ros2 command

```
ros2 run openscenario_utility yaml2xosc --input /path/to/scenario.yaml --output /path/to/output/directory
```
