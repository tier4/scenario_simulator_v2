# How to write workflow file
The workflow file defines how to execute scenarios and its expected results.

## Supported parameters

| name | type | required | description | default | example |
| ---- | ---- | -------- | ----------- | ------- | ------- |
| path | string | yes | Path to the .xosc (OpenSCENARIO .xml file.) file or T4V2 .yaml file. | | [example](#path) |
| expect | string | no | Only success/failure/exception values are support. Scenario writers can define the scenarios should be end with expected results. | success | [example](#expect) |
| step_time_ms | int | yes | step_time_ms describes the step time of the simulation in milliseconds. | 2 | [example](#step_time_ms) |

## Examples of writing parameters
### path

You can solve relative path from the "share" directories in your ROS2 packages.
```yaml
path: $(find-pkg-share PACKAGE_NAME)/test/scenario/simple.xosc
```
Or, you can solve absolute path like this way.
```yaml
path: /tmp/simple.xosc
```

### expect

```yaml
success: Simulation terminated by success condition action.
failure: Simulation terminated by failure condition action.
exception: Simulation terminated by exceptions in openscenario_intertretor component.
```

## Examples
```yaml
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
      step_time_ms: 2
    }
```
