# scenario_test_runner package

Scenario Test Runner is being developed to assist in the definitive planning
simulation using concept of OpenSCENARIO.
Simulations are described in a "YAML" based format called a "tier4 scenario format".
Then convert scenario into a "XML" based format called a "OpenSCENARIO" The format has been found at [OpenSCENARIO](http://www.openscenario.org/)


# How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

# Build with docker image

```
sh save_image.sh
docker import 
```


# Tier4 Format V2 -> OpenSCENARIO Format

### Scenario Modifiers
ScenarioModifiers and OpenSCENARIO is defined structure below
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
name express a variable It is not case sensitive, but attributes must be a lower snake case and it is converted to a variable in it's list during parameter distribution.
See more details in test folder

start,step stop express it's variable range.
initial parameter distribution is from start to end while increasing a value.

### Scenario Tags
CatalogLocations and other Tags inside OpenSCENARIO is defined structure below
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

attension or int
- if step is one, only parameter of start is used
- if step is zero, it returns error
- the number of simulation is factorial to number of  steps

## For None Value Expression

### OK
Without modifier element case
```
ScenarioModifiers:
or
ScenarioModifiers:{}
```

### NG
Neither Lack of neccesary keys nor [] is invaild
```
ScenarioModifiers:
  ScenarioModifier:
    - name: <String>
or

ScenarioModifiers:
  ScenarioModifier:[]
```


## Workflow Example
requirement
- path

options
- expectation
- step_time_ms


```
Scenario:
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/xosc/simple.xosc
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/yaml/failure.yaml,
      expect: failure
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/yaml/success.yaml,
      expect: success,
      step_time_ms: 2
    }
```

### Tier4 Format V2 -> OpenSCENARIO Format Separatory

To convert OpenSCENARIO, see scenario test utility packcage[Scenario Converter](../scenario_test_utility)


