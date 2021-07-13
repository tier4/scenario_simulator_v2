# scenario_test_runner package

Scenario Test Runner is being developed to assist in the definitive planning
simulation using concept of OpenSCENARIO.
Simulations are described in a "YAML" based format called a "TierIV Scenario Format".
Then convert the scenario into an "XML" based format called an "OpenSCENARIO". The details of the format can be found at [OpenSCENARIO](http://www.openscenario.org/).


# How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml'
```

# Build with docker image

```
sh save_image.sh
docker import
```


# TierIV Scenario Format version 2.0 -> OpenSCENARIO Format

### Scenario Modifiers
ScenarioModifiers and OpenSCENARIO are defined as below.
```
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

### Scenario Tags
CatalogLocations and other Tags inside OpenSCENARIO are defined as below.
```
CatalogLocations:
or
CatalogLocations: {}
```

## Parameter Distribution Example
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

## For None Value Expression

### OK
You can write the scenario as below if ScenarioModifiers is empty.
```
ScenarioModifiers:
or
ScenarioModifiers: {}
```

### Bad
You cannot write the scenario as below, because there is no necessary key
and `ScenarioModifier: []` is invalid syntax.
```
ScenarioModifiers:
  ScenarioModifier:
    - name: <String>
or

ScenarioModifiers:
  ScenarioModifier: []
```

## Workflow Example
Required
- path

Optional
- expect
- frame-rate

```
Scenario:
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/simple.xosc
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/failure.yaml,
      expect: failure
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/success.yaml,
      expect: success,
      frame-rate: 30
    }
```

### TierIV Scenario Format version 2.0 -> OpenSCENARIO Format Separator

To convert OpenSCENARIO, see the scenario test utility package [Scenario Converter](../scenario_test_utility).
