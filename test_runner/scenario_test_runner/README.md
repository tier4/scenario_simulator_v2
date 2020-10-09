# autoware.iv scenario_test_runner

Scenario Launcher is being developed to assist in the definitive planning
simulation using concept of open scenario.

Simulations are described in a "YAML" based format called a "tier4 scenario format".

Then convert scenario into a "XML" based format called a "open scenario" The format has been found at [Open Scenario](http://www.openscenario.org/)


# Scenario Preprocessor

## Scenario Explanation

ScenarioModifiers and OpenScenario is defined structure below
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

start,step stop express it's variable range.
initial parameter distribution is from start to end while increasing a value.

## Operation Example
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

## Abbreviation

### OK
Without modifier element case
```
ScenarioModifiers:
  ScenarioModifier:    
```
Without modifier case
```
ScenarioModifiers:
```

### NG
Without any replacement case
```
ScenarioModifiers:
  ScenarioModifier:
    - name: <String>
```


### How To Build
when building use these commands below
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --cmake-clean-cache --cmake-clean-first --packages-select scenario_test_runner scenario_test_utility scenario_runner_mock --symlink-install
```

### How To Test
test by using scenario runner mock
```
colcon build --symlink-install --packages-select scenario_test_runner scenario_test_utility scenario_runner_mock
source /path/to/install/local_setup.bash
ros2 launch scenario_test_runner dummy_runner.launch.py
```

### How To Run
to run scenario test runner use these commands below
```
source /path/to/install/local_setup.bash
ros2 run scenario_test_runner scenario_test_runner
```


### Tier4 Format V2 -> Open Scenario Format

To convert open scenario, see scenario test utility packcage[Scenario Converter](https://github.com/tier4/scenario_simulator.auto/tree/master/scenario_test_utility)


### Requirements



### add lines below to the scenario_database.yaml

```
Launch: "path/to/scenario_test_runner.launch"
Log: "path/to/log"
Map:
  - map1: "path/to/map1"
  - map2: "path/to/map2"
Scenario:
  - "path/to/scenario1.yaml"
  - "path/to/scenario2.yaml"
```
