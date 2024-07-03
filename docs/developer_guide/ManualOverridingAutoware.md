# Manual Override Simulation for Autoware

`scenario_simulator_v2` simulates the manual overriding of Autoware, with `FollowTrajectoryAction`.
During the executing `FollowTrajectoryAction`, the control of the ego entity is taken over from Autoware to the `FollowTrajectoryAction`.

## two triggers to override Autoware

There are 2 triggers to override Autoware.
One is hardware triggers like steering and brake pedal triggered by safety operators physically.

Another one is software trigger via services, like remote override and override with rviz button.

These triggers are received by vehicle interface in normal autoware driven autonomous vehicle.

## override simulation in scenario_simulator_v2

vehicle interface simulation is a part of ego vehicle simulation feature in `scenario_simulator_v2`.
`scenario_simulator_v2` simulates hardware override triggered by safety operators physically when a scenario commands overriding the ego vehicle by `FollowTrajectoryAction`.

## steps scenario_simulator_v2 takes to simulate the overrides  

### 1. triggering the override

In real vehicle, the override detected in vehicle internally and communicated to vehicle interface node such as `pacmod_interface`.

In `scenario_simulator_v2`, `openscenario_interpreter` send override flag via zmq interface between `traffic_simulator` and `simple_sensor_simulator` when `FollowTrajectoryAction` is started.

`simple_sensor_simulator` receives it and set control mode to MANUAL like vehicle interface do when hardware override triggers detected.

### 2. during the override

`traffic_simulator` send ego status calculated to follow described in the scenario and `simple_sensor_simulator` overrides Autoware control with overwriting ego status by the received ego status.

### 3. finishing the override

When `FollowTrajectoryAction` is finished, `traffic_simulator` call service to enable autoware control and stop sending override flag to `simple_sensor_simulator` via zmq communication.

This mimics the steps safety operators do in real vehicle via some human interfaces, in API level.
