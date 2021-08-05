# Release Notes

## Difference between the latest release and master
- Enables vehicle entity yeild to merging entity. See also [this video.](https://user-images.githubusercontent.com/10348912/128287863-8a2db025-d1af-4e54-b5a3-08e5d2e168e4.mp4)

## Version 0.4.2
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.2) on Github :fa-github:
- Fix problems in coordinate conversion from world to lane in pedestrian entity.
- Adding include_crosswalk option to the HdMapUtils::getClosetLaneletId() and HdMapUtils::toLaneletPose()

## Version 0.4.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.1) on Github :fa-github:
- Fix problem in follow front entity action, velocity planner was ignored requested target speed.

## Version 0.4.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.0) on Github :fa-github:
- Support OpenSCENARIO 1.0 TrafficSignal features (RoadNetwork.TrafficSignalController, Action and Condition).
- Update AcquirePositionAction to support WorldPosition as destination.
- Update syntax 'RoadNetwork.LogicFile' to allow user to specify the directory that contains `lanelet2_map.osm`.
- Check boost::none in TargetSpeedPlanner class.
- Add ROS2 galactic support.
- Update EgoEntity to publish self-position as PoseWithCovarianceStamped.

## Version 0.3.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.3.0) on Github :fa-github:
- Enable spawn MiscObjectEntity by using API class.
- Integrate with AutowareAuto (Autoware type is chosen at build time using `AUTOWARE_ARCHITECTURE_PROPOSAL` or `AUTOWARE_AUTO` flag). (Contribution by [Robotec.ai](https://robotec.ai/)).
- Update WorldPosition to be convertible with `openscenario_msgs::msg::LaneletPose`.
- Fix problems when the whole route is empty in route planner class.
- Support OpenSCENARIO 1.0 MiscObject.

## Version 0.2.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.2.0) on Github :fa-github:
- Enhance `/simulation/context` topic information. (adding TriggeringEntitiesRule, TriggeringEntities, CollisionCondition, TimeHeadwayCondition, AccelerationCondition, StandStillCondition, SpeedCondition, ReachPositionCondition, DistanceCondition, RelativeDistanceCondition, ParameterCondition, StoryboardElementStateCondition).
- NPC becomes unable to change lanes behind of them.

## Version 0.1.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.1.1) on Github :fa-github:
- Add support for RelativeTargetSpeed, the syntax of OpenSCENARIO.
- Add feature to publish context information during scenario execution to topic `/simulation/context` as a JSON string.
- Enable send warnings semantic error when you call setEntityStatus or setTargetSpeed function which targets to the ego vehicle after starting scenario.

## Version 0.1.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.1.0) on Github :fa-github:
- Synchronize ROS time between `traffic_simulator` and `sensor_simulator`.
- `openscenario_interpreter` now stores recorded rosbags for each simulation in `output_directory` (argument of `scenario_test_runner`).
- Add AutowareError, SemanticError, SimulationError, SpecificationViolation, SyntaxError and use these errors in `traffic_simulator` and `openscenario_interpreter` package.
- Remove old errors in `traffic_simulator` packages.
- Change Header ID of the lidar/detection sensor. (before : name of the entity -> after : `base_link`)
- Fix problems in publishing detection result. Before this change, the value of the pose was always (x, y, z, roll, pitch, yaw) = (0, 0, 0, 0, 0, 0).
- Remove unused packages (`joy_to_vehicle_cmd`, `scenario_runner_mock`).
- Fix problems when simulator running in 30 FPS and with 10 FPS sensors.
- Enable caching routing result, and center points and it' spline, lanelet length in hdmap_utils class.
- Update interpreter to access TrafficSignals from Action / Condition.
- Update EgoEntity to use precise simulation model parameters.
- Add getVehicleCommand function to the API class.

## Version 0.0.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.0.1) on Github :fa-github:
- Partially support OpenSCENARIO 1.0.0 format.
- Support Tier IV Scenario Format v2.
