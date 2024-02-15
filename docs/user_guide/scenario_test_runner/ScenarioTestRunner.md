# Scenario Test Runner User Guide

You can test run (preview) the scenarios you created using the GUI scenario editor before running it with Autoware.

The file you exported from the GUI scenario editor is in YAML based format called "[TIER IV Scenario Format Version 2.0](../../developer_guide/TIERIVScenarioFormatVersion2.md)". Then it is converted into XML based OpenSCENARIO format.
The specification of this OpenSCENARIO format is found at [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario-xml/) site.

## Before Testing Scenarios

You need to open your YAML file with text editor and edit the following two parts of your scenario file before testing it.

#### 1. Add Full Path to your Map File

Find the following part in your YAML file;

```bash
RoadNetwork:
    LogicFile:
      filepath: lanelet2_map.osm
```

and add the full path to your Lanelet2 map file based on your local setup. For example;

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
This is the setting to let Ego vehicle run without connecting to Autoware for testing the scenario. <font color="Red"> Please note that this "isEgo" setting is only for scenario testing, and you will need to revert it to "true" when you conduct the actual scenario simulation with Autoware. </font>

## How to Test Single Scenario
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:='/home/user-name/scenario-folder/t4v2.yaml' launch_rviz:=true
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](./HowToWriteWorkflowFile.md)

## How to Test Multiple Scenarios
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml' log_directory:='/tmp'
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](./HowToWriteWorkflowFile.md)


## Detailed Documentations

[How to write workflow file](./HowToWriteWorkflowFile.md)

[Scenario conversion](./ScenarioFormatConversion.md)

[Tips](./Tips.md)
