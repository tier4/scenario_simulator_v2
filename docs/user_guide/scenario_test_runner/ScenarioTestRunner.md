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

```yaml
RoadNetwork:
    LogicFile:
      filepath: /home/user-name/maps/location-a/lanelet2_map.osc
```

#### 2. Modify "isEgo" Setting

Find the following part in your YAML file;

```yaml
              Property:
                - name: isEgo
                  value: "true"
```

and change to;

```yaml
              Property:
                - name: isEgo
                  value: "false"
```

This is the setting to let Ego vehicle run without connecting to Autoware for testing the scenario.

!!! warning
    Please note that this "isEgo" setting is only for scenario testing, and you will need to revert it to "true" when you conduct the actual scenario simulation with Autoware.

## How to Test Single Scenario

```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:='/home/user-name/scenario-folder/t4v2.yaml' launch_rviz:=true
```

## How to Test Multiple Scenarios

You can write a simple script to run multiple scenarios like [this](https://github.com/tier4/scenario_simulator_v2/blob/e9376aa13a517da83d7ccb4e16f3d8919429ccec/.github/workflows/workflow.sh). (The sample usage is [here](https://github.com/tier4/scenario_simulator_v2/blob/e9376aa13a517da83d7ccb4e16f3d8919429ccec/.github/workflows/BuildAndRun.yaml#L129))

## Detailed Documentations

[Scenario conversion](./ScenarioFormatConversion.md)

[Tips](./Tips.md)
