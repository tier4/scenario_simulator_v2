# Scenario Test Runner User Guide

Scenario Test Runner is being developed to assist in the definitive planning simulation using the concept of OpenSCENARIO.
Simulations are described in a "YAML" based format called a "tier4 scenario format".
Then the scenario is converted into an XML based format called "OpenSCENARIO".
The specification of this format is found at [OpenSCENARIO](http://www.openscenario.org/).

## Before Testing Scenarios

You need to open your YAML file and edit the following two parts of your scenario file before testing it.

#### 1. Add Full Path to your map file 

Find the following part in your YAML file;

```bash
RoadNetwork:
    LogicFile:
      filepath: lanelet2_map.osm
```

and add the full path to your Lanelet2 map file based on your local setp. For example;

```bash
RoadNetwork:
    LogicFile:
      filepath: /home/user-name/maps/location-a/lanelet2_map.osc
```

#### 2. Modify "isEgo" setting

Find the following part in your YAML file;
```bash
              Property:
                - name: isEgo
                  value: "true"
```
and change to;

```bash
              Property:
                - name: isEgo
                  value: "false"
```
This is the setting to let Ego vehicle run without connecting to Autoware for test run. <font color="Red"> Please note that this "isEgo" setting is only for scenario testing and you will need to revert it to "true" when you conduct the actual sceario simulation with Autoware. </font>

## How to Test your Single Scenario
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:='/home/user-name/scenario-folder/t4v2.yaml' with_rviz:=true
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](HowToWriteWorkflowFile)

## How to Test your Mutiple Scenarios
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml' log_directory:='/tmp'
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](HowToWriteWorkflowFile)


## Detailed Documentations

[How to write workflow file](./HowToWriteWorkflowFile.md)

[Scenario conversion](./ScenarioFormatConversion.md)

[Tips](./Tips.md)
