# Release Notes

## Difference between latest release and master
- Synchronize ROS time between traffic_simulator and sensor_simulator.
- openscenario_interpreter now stores recorded rosbags for each simulation in output_directory (argument of scenario_test_runner).
- Add AutowareError, SemanticError, SimulationError, SpecificationViolation, SyntaxError and use these errors in traffic_simulator and openscenario_interpretor package.
- Remove old errors in traffic_simulator packages.
- Change Header ID of the lidar/detection sensor. (before : name of the entity -> after : base_link)
- Fix problems in publishing detection result. Before this change, the value of the pose was always (x,y,z,roll,pitch,yaw) = (0,0,0,0,0,0)
- Remove unused packages (joy_to_vehicle_cmd, scenario_runner_mock)

## Ver 0.0.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.0.1) on Github :fa-github:
- Partially support OpenSCENARIO 1.0.0 format
- Support Tier IV Scenario Formar v2
