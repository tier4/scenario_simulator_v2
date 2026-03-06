# Scenario Format Conversion

### TIER IV Format V2 -> OpenSCENARIO Format

To convert OpenSCENARIO, use these arguments

| Input  | Required | Description                                                                                                                                           |
|--------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| input  | yes      | TIER IV format version2 yaml files in a specified directory, reference directory is relative to the current directory.                                |
| output | no       | TIER IV parameter distributed OpenSCENARIO .xosc files are generated in a specified directory. Default is under the current directory converted_xosc. |

You can execute scenario conversion by using ros2 command

```bash
ros2 run openscenario_utility conversion.py --input /path/to/scenario.yaml --output /path/to/output/directory
```

### Scenario Modifiers

ScenarioModifiers and OpenSCENARIO are defined as below.

```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: <string>
      list: [<any>, <any> ..., <any>]
    - name: <string>
      start: <float or int>
      step: <float or int>
      stop: <float or int>
OpenSCENARIO:
  FileHeader: <string>
    revMajor: <string or int>
    revMinor: <string or int>
    date: <string or int>
    description: <string>
    author: <string>
  ParameterDeclarations:
  .
  .
```

`name` expresses a variable: It is not case-sensitive, but its attributes must be the lower snake case, and it is converted to a variable in its list during parameter distribution.
You can find more details in the test folder.

`start`, `step`, `stop` express theirs variable ranges:
Initial parameter distribution is from start to end while increasing a value.

### Parameter Distribution Example

ex)
start: 10
stop: 20
if step is 1 then distributed parameter is 10
if step is 2 distributed parameter is 10 , 20
if step is 3 distributed parameter is 10 , 15, 20

attention or int

- if step is one, only parameter of start is used
- if step is zero, it returns error
- the number of simulation is factorial to number of  steps
