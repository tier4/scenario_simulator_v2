# How To Build

```
colcon build --symlink-install --packages-select scenario_test_runner scenario_runner_mock
colcon test-result --all
```

source 


# How To Run
```
ros2 run scenario_runner_mock scenario_runner_mock
ros2 run scenario_test_runner lifecycle_controller
```

