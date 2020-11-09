# ScenarioSimulator

![ROS2-Dashing](https://github.com/tier4/scenario_simulator.auto/workflows/ROS2-Dashing/badge.svg)

Open scenario interpreter and simple simulator for Autoware.auto 

![rviz](image/rviz.png "rviz")


# How to use
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

# Run with docker image

```
docker pull tier4/scenario_simulator:latest
docker run -p 6080:80hm-size=512m scenario_simulator
```

launch lx terminal

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```