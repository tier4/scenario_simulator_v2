# Manual Override Simulation with FollowTrajectoryAction

`scenario_simulator_v2` simulates the manual override of Autoware, with `FollowTrajectoryAction`.
During the executing `FollowTrajectoryAction`, the control of the ego entity is taken over from Autoware to the `FollowTrajectoryAction`.

## 2 triggers to override Autoware

There are 2 triggers to override Autoware.
One is hardware triggers like a steering and a brake pedal, physically triggered by safety operators.

Another one is a software trigger via ROS 2 services or topics, like a remote override.

These triggers are received by vehicle interface in autoware driven autonomous vehicles.

## override simulation in scenario_simulator_v2

vehicle interface simulation is a part of the ego vehicle simulation feature in `scenario_simulator_v2`.
`scenario_simulator_v2` simulates a hardware override triggered by safety operators when a scenario commands overriding the ego vehicle by `FollowTrajectoryAction`.

## 3 steps scenario_simulator_v2 takes to simulate the overrides

### 1. triggering the override

In real vehicle, the override detected in vehicle internally and communicated to vehicle interface node such as `pacmod_interface` node.

In `scenario_simulator_v2`, `openscenario_interpreter` send an override flag via zmq interface between `traffic_simulator` and `simple_sensor_simulator` when `FollowTrajectoryAction` is started.

`simple_sensor_simulator` receives it and set the control mode to MANUAL like vehicle interface do when hardware override triggers detected.

### 2. during the override

`traffic_simulator` send ego status calculated to follow described in the scenario and `simple_sensor_simulator` overrides Autoware control with overwriting ego status by the received ego status.

### 3. finishing the override

When `FollowTrajectoryAction` is finished, `traffic_simulator` call service to enable autoware control and stop sending the override flag to `simple_sensor_simulator` via zmq communication.

This mimics the steps safety operators do in real vehicle via some human interfaces, in API level.
