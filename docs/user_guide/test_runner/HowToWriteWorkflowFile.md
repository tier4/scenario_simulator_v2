# How to write workflow file


parameters
```
required
- path

options
- expect : success/failure/exception
- step_time_ms : float value
```

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