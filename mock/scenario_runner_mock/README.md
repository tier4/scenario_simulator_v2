# How To Build

```
colcon build --symlink-install --packages-select scenario_launcher scenario_runner_mock
colcon test-result --all
```

source 


# How To Run
```
ros2 run scenario_runner_mock scenario_runner_mock
ros2 run scenario_launcher lifecycle_controller
```

