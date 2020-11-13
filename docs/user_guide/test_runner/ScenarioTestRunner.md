# Scenario Test Runner User Guide

Scenario Test Runner is being developed to assist in the definitive planning
simulation using concept of OpenSCENARIO.
Simulations are described in a "YAML" based format called a "tier4 scenario format".
Then convert scenario into a "XML" based format called a "OpenSCENARIO" The format has been found at [OpenSCENARIO](http://www.openscenario.org/)


## How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

### Scenario Tags
CatalogLocations and other Tags inside OpenSCENARIO is defined structure below
```
CatalogLocations:
or
CatalogLocations: {}
```

## Workflow Example

parameters
```
requirement
- path

options
- expect : success/failure/exception
- step_time_ms : float value
```

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

To convert OpenSCENARIO, see scenario test utility packcage[Scenario Converter](ScenarioFormatConversion.md)