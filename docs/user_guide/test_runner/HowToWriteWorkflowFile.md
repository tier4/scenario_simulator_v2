# How to write workflow file
Workflow file defines how to execute scenarios and it's expected results.

## Support parameters
### path
type : string value  
required : true  

Path to the .xosc (OpenSCENARIO .xml file.) file or T4V2 .yaml file.  
We can solve the ros2 package share directory path by this way.  
```yaml
path: $(find-pkg-share PACKAGE_NAME)/test/scenario/xosc/simple.xosc
```
or, you can solve absolute path like this way.
```yaml
path: /tmp/simple.xosc
```

### expect
type : string value  
required : false  
default value : success  

Only success/failure/exception values are support.  
Scenario writers can define the scenarios should be end with expected results.  
```yaml
success: Simulation terminated by success condition action.  
failure: Simulation terminated by failure condition action.  
exception: Simulation terminated by exceptions in openscenario_intertretor component.  
```

### step_time_ms
type : int value
required : false  
default value : 2  

step_time_ms describes the step time of the simulation in milliseconds.

## example
```yaml
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