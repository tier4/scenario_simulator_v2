# How to write workflow file
The workflow file defines how to execute scenarios and its expected results.

## Supported parameters

| name         | type   | required | description                                                             | default | example                  |
| ------------ | ------ | -------- | ----------------------------------------------------------------------- | ------- | ------------------------ |
| path         | string | yes      | Path to the .xosc (OpenSCENARIO .xml file.) file or T4V2 .yaml file.    |         | [example](#path)         |
| step_time_ms | int    | yes      | step_time_ms describes the step time of the simulation in milliseconds. | 2       | [example](#step_time_ms) |

[//]: # (TODO: resolve example link in step_time_ms raw)
## Examples of writing parameters
### path

You can solve relative path from the "share" directories in your ROS 2 packages.
```yaml
path: $(find-pkg-share PACKAGE_NAME)/test/scenario/simple.xosc
```
Or, you can solve absolute path like this way.
```yaml
path: /tmp/simple.xosc
```

## Examples
```yaml
Scenario:
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/simple.xosc
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/minimal.yaml
    }
  - {
      path: $(find-pkg-share scenario_test_runner)/test/scenario/success.yaml,
      step_time_ms: 2
    }
```
