# Release Notes

## Difference between the latest release and master
- Start supporting linear trajectory shape while changing lane. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/661))
- Fix context panel to display simulation time. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Support new vehicle_model_type `DELAY_STEER_VEL` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/660)).

## Version 0.6.1
- Add API::requestSpeedChange function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/618))
- Fix syntax `Controller` to not to overwrite `traffic_simulator`'s `DriverModel` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/649)).
- Update simulation models ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/652)).
- Enable request relative speed change. ([link](https://github.com/tier4/scenario_simulator_v2/pull/654))

## Version 0.6.0
- Start supporting Autoware.Universe. ([link](https://github.com/tier4/scenario_simulator_v2/pull/614))

## Version 0.5.8
- Remove newton methods in getSValue function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/612))
- Set withLaneChange parameter as false. ([link](https://github.com/tier4/scenario_simulator_v2/pull/618))
- Change traffic light topic name to "/perception/traffic_light_recognition/traffic_light_states" ([link](https://github.com/tier4/scenario_simulator_v2/pull/621))
- Remove hard coded parameters in behavior tree plugin and use acceleration and deceleration value in traffic_simulator_msgs/msg/DriverModel. ([link](https://github.com/tier4/scenario_simulator_v2/pull/624))
- Enable build with foxy latest version of behavior_tree_cpp_v3. ([link](https://github.com/tier4/scenario_simulator_v2/pull/625))
- Add API::getDriverModel function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/626))
- Add Random test runner. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/619) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Support new Autoware architecture `Universe` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/614)).

## Version 0.5.7
- Fix problem in getNormalVector function. (Contribution by [Utaro-M](https://github.com/Utaro-M)).

## Version 0.5.6
- Fix context panel to display conditions' status. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Add NPC Behavior Plugin and Behavior-Tree Plugin for Vehicle and Pedestrian. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/566))
- Rename package `openscenario_msgs` to `traffic_simulator_msgs`
- Start supporting galactic environment with Docker. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/576))
- Update `UserDefinedValueCondition` to subscribe ROS2 topic if path-like name given ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/567)).
- Add new package `openscenario_msgs` ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/567)).
- Add getNearbyLaneletIds and filterLaneletIds function in HdMapUtils class. ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/585))
- Fix calculating way of longitudinal distance. If forward distance and backward distance was calculated, choose smaller one. ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/586))

## Version 0.5.5
- Fix syntax `ReachPositionCondition` to not to use `API::reachPosition` ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/561)).

## Version 0.5.4
- Revert [PR #544](https://github.com/tier4/scenario_simulator_v2-docs/pull/544) ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/557))
- Add context panel to display conditions' status. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Add Tier IV extension `conditionEdge="sticky"` to `OpenSCENARIO-1.1.xsd` ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/553)).
- Add new Controller's Property `maxSpeed` to set explicitly upper bound speed to Autoware ([pull request](https://github.com/tier4/scenario_simulator_v2-docs/pull/554)).

## Version 0.5.3
- Fix setStatus function in EgoEntity class. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/549))

## Version 0.5.2
- Remove entity status which is empty. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/544)).
- Update syntaxes of `openscenario_interpreter` to be compiled separately ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/532)).

## Version 0.5.1
- Support OpenSCENARIO 1.1 `AddEntityAction` ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/506)).
- Add StandStillMetric. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/520)
- Add CollisionMetric. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/521)
- Support OpenSCENARIO 1.1 `RelativeDistanceCondition` ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/519)).
- Fixed Action to not cause side effects during `startTransition` ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/522)).
- Fixed log directory cleaning behavior, cleaning all files and directories under log directory without deleting itself. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/527)).
- Support new `CustomCommandAction` type `FaultInjectionAction` for ArchitectureProposal ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/491))
- Enable run cpp_mock_test with colcon test (with -DWITH_INTEGRATION_TEST=ON) ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/529))
- Support OpenSCENARIO 1.1 `DistanceCondition` ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/533)).
- Fix problems in getLongitudinalDistance function when the target entity does not matched to the lane. ([link](https://github.com/tier4/scenario_simulator_v2-docs/pull/536)).

## Version 0.5.0
- Add arrow markers to visualize goal poses of entities. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Fix problems in setting entity names in proto message. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/481) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in never hit line. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/480) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Start support getting longitudinal distance to the behind entity in API::getLongitudinalDistance function. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/486)
- Fix problems in update/getPhaseDuration function in traffic light class. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/492)
- Update dependency for message types of Autoware to reference `AutowareArchitectureProposal_msgs` instead of `AutowareArchitectureProposal.iv`.
- Add to the AutowareArchitectureProposal_api_msgs to the .repos file. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/496)
- Fix rotation calculation in toMapPose function. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/500)
- Simplify logics in bool API::spawn(const bool is_ego, const std::string & name, const openscenario_msgs::msg::VehicleParameters & params) [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/486) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix lane coordinate calculation logic for pedestrian in walk straight action. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/507)

## Version 0.4.5
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.5) on Github :fa-github:
- Add `move_backward` scenario. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/461)
- Fix problems in `CppScenarioNode::start()` function, `onInitialize()` function was called before starting simulation (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in initializing `current_time` value in entity manager class (Contribution by [Robotec.ai](https://robotec.ai/)).
- Supports the option (--architecture-type) to select between AutowareArchitectureProposal and Autoware.Auto (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in HdMap::toMapPose function, offset distance was calculated from tangent vector. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/476)
- Support new options `initialize_duration:=<int>`, `launch_autoware:=<boolean>` and `record:=<boolean>`.

## Version 0.4.4
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.4) on Github :fa-github:
- Add a new metrics module which detects vel, acc and jerk out of range (Contribution by [kyabe2718](https://github.com/kyabe2718)).
- Fix phase control feature in traffic light manager class. [Link](https://github.com/tier4/scenario_simulator_v2-docs/pull/450)
- Fix problems in crossing entity on crosswalk. [link](https://github.com/tier4/scenario_simulator_v2-docs/pull/452)

## Version 0.4.3
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.3) on Github :fa-github:
- Checking collision between crosswalk and waypoints in lane coordinate.
- Remove division in checking collision function in order to avoid zero-division.
- Enables vehicle entity yield to merging entity. See also [this video](https://user-images.githubusercontent.com/10348912/128287863-8a2db025-d1af-4e54-b5a3-08e5d2e168e4.mp4).
- Simplify the contents of the scenario test result file `result.junit.xml`.

## Version 0.4.2
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.2) on Github :fa-github:
- Fix problems in coordinate conversion from world to lane in pedestrian entity.
- Adding `include_crosswalk` option to the HdMapUtils::getClosestLaneletId() and HdMapUtils::toLaneletPose()

## Version 0.4.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.1) on Github :fa-github:
- Fix problem in follow front entity action, velocity planner was ignored requested target speed.

## Version 0.4.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.4.0) on Github :fa-github:
- Support OpenSCENARIO 1.0 TrafficSignal features (RoadNetwork.TrafficSignalController, Action and Condition).
- Update AcquirePositionAction to support WorldPosition as destination.
- Update syntax 'RoadNetwork.LogicFile' to allow user to specify the directory that contains `lanelet2_map.osm`.
- Check boost::none in TargetSpeedPlanner class.
- Add ROS2 galactic support.
- Update EgoEntity to publish self-position as PoseWithCovarianceStamped.

## Version 0.3.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.3.0) on Github :fa-github:
- Enable spawn MiscObjectEntity by using API class.
- Integrate with AutowareAuto (Autoware type is chosen at build time using `AUTOWARE_ARCHITECTURE_PROPOSAL` or `AUTOWARE_AUTO` flag). (Contribution by [Robotec.ai](https://robotec.ai/)).
- Update WorldPosition to be convertible with `openscenario_msgs::msg::LaneletPose`.
- Fix problems when the whole route is empty in route planner class.
- Support OpenSCENARIO 1.0 MiscObject.

## Version 0.2.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.2.0) on Github :fa-github:
- Enhance `/simulation/context` topic information. (adding TriggeringEntitiesRule, TriggeringEntities, CollisionCondition, TimeHeadwayCondition, AccelerationCondition, StandStillCondition, SpeedCondition, ReachPositionCondition, DistanceCondition, RelativeDistanceCondition, ParameterCondition, StoryboardElementStateCondition).
- NPC becomes unable to change lanes behind of them.

## Version 0.1.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.1.1) on Github :fa-github:
- Add support for RelativeTargetSpeed, the syntax of OpenSCENARIO.
- Add feature to publish context information during scenario execution to topic `/simulation/context` as a JSON string.
- Enable send warnings semantic error when you call setEntityStatus or setTargetSpeed function which targets to the ego vehicle after starting scenario.

## Version 0.1.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.1.0) on Github :fa-github:
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
- [Release Page](https://github.com/tier4/scenario_simulator_v2-docs/releases/0.0.1) on Github :fa-github:
- Partially support OpenSCENARIO 1.0.0 format.
- Support Tier IV Scenario Format v2.
