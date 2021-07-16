# Scenario Test Runner User Guide

Scenario Test Runner is being developed to support the scenario simulation using the OpenSCENARIO format.
Scenarios are described in the YAML based format called "tier4 scenario format".
Then the scenario is converted into XML based OpenSCENARIO format.
The specification of this scenario format is found at [OpenSCENARIO](http://www.openscenario.org/) site.

## Before Testing Scenarios

You need to open your YAML file with text editor and edit the following two parts of your scenario file before testing it.

#### 1. Add Full Path to your Map File 

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

#### 2. Modify "isEgo" Setting

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
This is the setting to let Ego vehicle run without connecting to Autoware for testing the scenario. <font color="Red"> Please note that this "isEgo" setting is only for scenario testing and you will need to revert it to "true" when you conduct the actual sceario simulation with Autoware. </font>

## How to Test Single Scenario
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:='/home/user-name/scenario-folder/t4v2.yaml' with_rviz:=true
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](HowToWriteWorkflowFile)

## How to Test Mutiple Scenarios
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml' log_directory:='/tmp'
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](HowToWriteWorkflowFile)


## Detailed Documentations

[How to write workflow file](./HowToWriteWorkflowFile.md)

[Scenario conversion](./ScenarioFormatConversion.md)

[Tips](./Tips.md)
