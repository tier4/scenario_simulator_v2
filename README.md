# ScenarioSimulator.Auto ![ROS2-Dashing](https://github.com/tier4/scenario_simulator.auto/workflows/ROS2-Dashing/badge.svg)

Open scenario interpreter and simple simulator for Autoware.auto 

# How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

# Build with docker image

```
docker build --build-arg GITHUB_USER=<github_username> --build-arg GITHUB_TOKEN=<github_token> -t scenario_simulator .
```

# How to use scenario editor
see [Scenario Editor](doc/README.md)

# How to use scenario test runner
see [Scenario Test Runner](test_runner/scenario_test_runner)
