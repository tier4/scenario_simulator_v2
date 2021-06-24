# Release Notes

## Difference between the latest release and master
- Add /simulator/context topic (openscenario_interpreter_msgs/msg/Context type) and enable visualize scenario execution status.
- NPC becomes ubable to change lanes behind of them.

## Ver 0.1.1
- Add support for RelativeTargetSpeed, the syntax of OpenSCENARIO
- Add feature to publish context information during scenario execution to topic `/simulation/context` as a JSON string
- Enable send warnings semantic error when you call setEntityStatus or setTargetSpeed function which targets to the ego vehicle after starting scenario.

## Ver 0.1.0
- Synchronize ROS time between traffic_simulator and sensor_simulator.
- openscenario_interpreter now stores recorded rosbags for each simulation in output_directory (argument of scenario_test_runner).
- Add AutowareError, SemanticError, SimulationError, SpecificationViolation, SyntaxError and use these errors in traffic_simulator and openscenario_interpreter package.
- Remove old errors in traffic_simulator packages.
- Change Header ID of the lidar/detection sensor. (before : name of the entity -> after : base_link)
- Fix problems in publishing detection result. Before this change, the value of the pose was always (x,y,z,roll,pitch,yaw) = (0,0,0,0,0,0)
- Remove unused packages (joy_to_vehicle_cmd, scenario_runner_mock)
- Fix problems when simulator running in 30 FPS and with 10 FPS sensors
- Enable caching routing reslut, and center points and it' spline, lanelet length in hdmap_utils class
- Update interpreter to access TrafficSignals from Action / Condition
- Update EgoEntity to use precise simulation model parameters
- Add getVehicleCommand function to the API class

## Ver 0.0.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.0.1) on Github :fa-github:
- Partially support OpenSCENARIO 1.0.0 format
- Support Tier IV Scenario Format v2
