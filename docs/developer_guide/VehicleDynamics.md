# Vehicle Dynamics

In this section, we describe the vehicle model which is implemented in scenario_simulator_v2.

## Vehicle Model Types

Pose is calculated by bicycle kinematics model.

Steering and velocity models are implemented 3 types of models.

### IDEAL_STEER

Uses velocity command. The steering and velocity vary ideally as commanded.

### IDEAL_ACCEL

Uses acceleration command. The steering and acceleration varies ideally as commanded.

### DELAY_STEER

Uses velocity command. The steering and velocity vary following a first-order delay model.

### DELAY_STEER_ACC

Uses acceleration command. The steering and acceleration vary following a first-order delay model.

## Vehicle Model Parameters

| Name                 | Type   | Description                                          | IDEAL_STEER | IDEAL_ACCEL | DELAY_STEER | DELAY_STEER_ACC | Default value | unit    |
|----------------------|--------|------------------------------------------------------|-------------|-------------|-------------|-----------------|---------------|---------|
| vel_time_delay       | double | dead time for the velocity input                     | x           | x           | o           | x               | 0.25          | [s]     |
| acc_time_delay       | double | dead time for the acceleration input                 | x           | x           | x           | o               | 0.1           | [s]     |
| steer_time_delay     | double | dead time for the steering input                     | x           | x           | o           | o               | 0.24          | [s]     |
| vel_time_constant    | double | time constant of the 1st-order velocity dynamics     | x           | x           | o           | x               | 0.61          | [s]     |
| acc_time_constant    | double | time constant of the 1st-order acceleration dynamics | x           | x           | x           | o               | 0.1           | [s]     |
| steer_time_constant  | double | time constant of the 1st-order steering dynamics     | x           | x           | o           | o               | 0.27          | [s]     |
| vel_lim              | double | limit of velocity                                    | x           | x           | o           | o               | 50.0          | [m/s]   |
| accel_rate           | double | limit of acceleration                                | x           | x           | o           | o               | 7.0           | [m/ss]  |
| steer_lim            | double | limit of steering angle                              | x           | x           | o           | o               | 1.0           | [rad]   |
| steer_rate_lim       | double | limit of steering angle change rate                  | x           | x           | o           | o               | 5.0           | [rad/s] |
| deadzone_delta_steer | double | dead zone for the steering dynamics                  | x           | x           | o           | o               | 0.0           | [rad]   |

_Note_: The steering/velocity/acceleration dynamics is modeled by a first-order system with a deadtime in a _delay_ model. The definition of the _time constant_ is the time it takes for the step response to rise up to 63% of its final value. The _deadtime_ is a delay in the response to a control input.

## Example Definition

```yaml
acc_time_constant: 0.1
acc_time_delay: 0.1
accel_rate: 7.0
add_measurement_noise: true
angvel_lim: 3.0
angvel_noise_stddev: 0.0
angvel_rate: 1.0
angvel_time_constant: 0.5
angvel_time_delay: 0.2
initial_engage_state: true
pos_noise_stddev: 0.01
rpy_noise_stddev: 0.0001
sim_steering_gear_ratio: 15.0
steer_lim: 1.0
steer_noise_stddev: 0.0001
steer_rate_lim: 5.0
steer_time_constant: 0.27
steer_time_delay: 0.24
tread_length: 1.0

vehicle_model_type: DELAY_STEER_ACC
vel_lim: 50.0
vel_noise_stddev: 0.0
vel_time_constant: 0.61
vel_time_delay: 0.25
```

This example shows DELAY_STEER_ACC model. If you want to use another type, please set another variable.
