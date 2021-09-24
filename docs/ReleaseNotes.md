# Release Notes

## Difference between the latest release and master
- Support OpenSCENARIO 1.1 `AddEntityAction` ([link](https://github.com/tier4/scenario_simulator_v2/pull/506)).
- Add StandStillMetric. [Link](https://github.com/tier4/scenario_simulator_v2/pull/520)
- Add CollisionMetric. [Link](https://github.com/tier4/scenario_simulator_v2/pull/521)
- Support OpenSCENARIO 1.1 `RelativeDistanceCondition` ([link](https://github.com/tier4/scenario_simulator_v2/pull/519)).
- Fixed Action to not cause side effects during `startTransition` ([link](https://github.com/tier4/scenario_simulator_v2/pull/522)).
- Support new `CustomCommandAction` type `FaultInjectionAction` for ArchitectureProposal ([link](https://github.com/tier4/scenario_simulator_v2/pull/491))
- Enable run cpp_mock_test with colcon test (with -DWITH_INTEGRATION_TEST=ON) ([link](https://github.com/tier4/scenario_simulator_v2/pull/529))

## Version 0.5.0
- Add arrow markers to visualize goal poses of entities. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Fix problems in setting entity names in proto message. [Link](https://github.com/tier4/scenario_simulator_v2/pull/481) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in never hit line. [Link](https://github.com/tier4/scenario_simulator_v2/pull/480) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Start support getting longitudinal distance to the behind entity in API::getLongitudinalDistance function. [Link](https://github.com/tier4/scenario_simulator_v2/pull/486)
- Fix problems in update/getPhaseDuration function in traffic light class. [Link](https://github.com/tier4/scenario_simulator_v2/pull/492)
- Update dependency for message types of Autoware to reference `AutowareArchitectureProposal_msgs` instead of `AutowareArchitectureProposal.iv`.
- Add to the AutowareArchitectureProposal_api_msgs to the .repos file. [Link](https://github.com/tier4/scenario_simulator_v2/pull/496)
- Fix rotation calculation in toMapPose function. [Link](https://github.com/tier4/scenario_simulator_v2/pull/500)
- Simplify logics in bool API::spawn(const bool is_ego, const std::string & name, const openscenario_msgs::msg::VehicleParameters & params) [Link](https://github.com/tier4/scenario_simulator_v2/pull/486) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix lane coordinate calculation logic for pedestrian in walk straight action. [Link](https://github.com/tier4/scenario_simulator_v2/pull/507)

## Version 0.4.5
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.5) on Github :fa-github:
- Add `move_backward` scenario. [Link](https://github.com/tier4/scenario_simulator_v2/pull/461)
- Fix problems in `CppScenarioNode::start()` function, `onInitialize()` function was called before starting simulation (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in initializing `current_time` value in entity manager class (Contribution by [Robotec.ai](https://robotec.ai/)).
- Supports the option (--architecture-type) to select between AutowareArchitectureProposal and Autoware.Auto (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in HdMap::toMapPose function, offset distance was calculated from tangent vector. [Link](https://github.com/tier4/scenario_simulator_v2/pull/476)
- Support new options `initialize_duration:=<int>`, `launch_autoware:=<boolean>` and `record:=<boolean>`.

## Version 0.4.4
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.4) on Github :fa-github:
- Add a new metrics module which detects vel, acc and jerk out of range (Contribution by [kyabe2718](https://github.com/kyabe2718)).
- Fix phase control feature in traffic light manager class. [Link](https://github.com/tier4/scenario_simulator_v2/pull/450)
- Fix problems in crossing entity on crosswalk. [link](https://github.com/tier4/scenario_simulator_v2/pull/452)

## Version 0.4.3
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.3) on Github :fa-github:
- Checking collision between crosswalk and waypoints in lane coordinate.
- Remove division in checking collision function in order to avoid zero-division.
- Enables vehicle entity yield to merging entity. See also [this video](https://user-images.githubusercontent.com/10348912/128287863-8a2db025-d1af-4e54-b5a3-08e5d2e168e4.mp4).
- Simplify the contents of the scenario test result file `result.junit.xml`.

## Version 0.4.2
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.2) on Github :fa-github:
- Fix problems in coordinate conversion from world to lane in pedestrian entity.
- Adding `include_crosswalk` option to the HdMapUtils::getClosetLaneletId() and HdMapUtils::toLaneletPose()

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
