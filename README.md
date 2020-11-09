# ScenarioSimulator

![ROS2-Dashing](https://github.com/tier4/scenario_simulator.auto/workflows/ROS2-Dashing/badge.svg)

Open scenario interpreter and simple simulator for Autoware.auto 

![rviz](image/rviz.png "rviz")


## How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

## Run with docker image

```
docker pull tier4/scenario_simulator:latest
docker run -p 6080:80 --shm-size=512m scenario_simulator
```

launch lx terminal

```
docker build --build-arg GITHUB_USER=<github_username> --build-arg GITHUB_TOKEN=<github_token> -t scenario_simulator .
```

## How to use scenario editor
See [Scenario Editor](doc/README.md)

## How to use scenario test runner
See [Scenario Test Runner](test_runner/scenario_test_runner)
