^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2022-01-11)
------------------
* Merge pull request `#655 <https://github.com/tier4/scenario_simulator_v2/issues/655>`_ from tier4/fix/galactic_build
* remove debug line
* remove glog functions
* remove glog from depends
* add glog and use unique_ptr
* add debug line
* add virtual destructor
* Merge pull request `#652 <https://github.com/tier4/scenario_simulator_v2/issues/652>`_ from tier4/feature/traffic_simulator/vehicle_model
* Lipsticks
* Cleanup some switch statements
* Cleanup `EgoEntity::makeSimulationModel`
* Remove message type `VehicleStateCommand` from VehicleModels
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Remove old VehicleModel `sim_model_ideal.[ch]pp`
* Merge pull request `#648 <https://github.com/tier4/scenario_simulator_v2/issues/648>`_ from tier4/feature/request_speed_change
* Merge pull request `#653 <https://github.com/tier4/scenario_simulator_v2/issues/653>`_ from tier4/fix/error_in_driver_model_from_blackboard
* fix typo
* remove recurrent call in setDriverModel function
* Restore AAP's VehicleModels
* apply reformat
* Fix `EgoEntity` to set `GearCommand` to VehicleModel
* add space
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change
* Lipsticks
* set default driver model in pedestrian entity class
* Merge pull request `#650 <https://github.com/tier4/scenario_simulator_v2/issues/650>`_ from tier4/fix/get_driver_model_in_pedestrian
* remove old header file
* Update VehicleModels to match latest `simple_planning_simulator_node`
* fix linter error
* Remove unused enumerations of `VehicleModelType`
* enable pass compile
* add const to the function
* fix way of calling API
* remove old API
* add requestSpeedChange function to the misc object and pedestrian
* change EntityBase::setDriverModel to the pure virtual function
* add data type
* modify test case
* add requestSpeedChange API
* Merge pull request `#646 <https://github.com/tier4/scenario_simulator_v2/issues/646>`_ from tier4/feature/set_acceleration_deceleration
* rename function
* rename function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge pull request `#628 <https://github.com/tier4/scenario_simulator_v2/issues/628>`_ from tier4/feature/avoid_overwrite_acceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* enable pass colcon test
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* add getDriverModel function in egoEntity class
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* add setAcceleration/Deceleration function to the API class
* add setAcceleration/Develeration to the vehicle entity class
* add setAcceleration/Deceleration function to the entity manager class
* add setAcceleration and setDeceleration to the base class
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge pull request `#640 <https://github.com/tier4/scenario_simulator_v2/issues/640>`_ from RobotecAI/fix/multi-lane-traffic-light-stopline-search
* multiple lane traffic light stopline search fix
* Merge pull request `#637 <https://github.com/tier4/scenario_simulator_v2/issues/637>`_ from tier4/feature/pass_goal_poses_to_the_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Merge pull request `#633 <https://github.com/tier4/scenario_simulator_v2/issues/633>`_ from tier4/feature/transform_point
* remove header
* add getGoalPosesInWorldFrame();
* Merge branch 'feature/transform_point' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* use const &
* Merge branch 'feature/transform_point' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* add transformPoint function
* move transform points to the transform.cpp and hpp
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* add key
* add getGoalPoses function to the plugin
* Break indentation (due to ament_clang_format)
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Remove `autoware_auto_msgs` from dependency
* Set default `architecture_type` to `tier4/proposal`
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Update `EntityManager` to select `TrafficLightManager` message type
* Update `TrafficLight(Arrow|Color)` to ROS2 message type conversion
* Update `TrafficLightManager` publisher to be parameterizable
* Merge pull request `#622 <https://github.com/tier4/scenario_simulator_v2/issues/622>`_ from tier4/fix-pointcloud-topic
* fix topic name of pointcloud
* Update `TrafficLightManager` to create publishers by itself
* Update class `SensorSimulation` to choice topic name and type based on Autoware's architecture type
* Add new virtual class `DetectionSensorBase`
* Update `API::attachDetectionSensor` to detect Autoware architecture
* Update `API::attachLidarSensor` to detect Autoware architecture
* Remove `autoware_auto_control_msgs.proto`
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Restore virtual function `EntityBase::getVehicleCommand`
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* Comment-out some tests and Remove protobuf type `GearCommand`, `GearReport`
* change no_ground pointcloud topic name
* Update DetectionSensor to use `autoware_auto_perception_msgs`
* Remove member function `getVehicleCommand` from Vehicle type entity
* use auto_msgs for traffic lights
* remove autoware_auto_msgs dependency
* some changes to run psim with autoware_universe
* Update some packages to use `tier4/autoware_auto_msgs`
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge pull request `#630 <https://github.com/tier4/scenario_simulator_v2/issues/630>`_ from tier4/feature/add_ideal_accel_model
* Merge commit 'ce08abe39ed83d7ec0630560d293187fbcf08b5e' into feature/add_ideal_accel_model
* Merge pull request `#631 <https://github.com/tier4/scenario_simulator_v2/issues/631>`_ from tier4/fix/set_driver_model
* Merge pull request `#619 <https://github.com/tier4/scenario_simulator_v2/issues/619>`_ from RobotecAI/AJD-254-simple_abstract_scenario_for_simple_random_testing
* disable throw errors while calling setDriverModel error
* add IDEAL_ACCEL model
* Merge remote-tracking branch 'tier/master' into AJD-254-simple_abstract_scenario_for_simple_random_testing
* Merge pull request `#626 <https://github.com/tier4/scenario_simulator_v2/issues/626>`_ from tier4/feature/get_driver_model
* add getDriverModel function
* fix compile errors
* move functions into .cpp file
* add getter setter to the base class
* Merge pull request `#623 <https://github.com/tier4/scenario_simulator_v2/issues/623>`_ from RobotecAI/fix/traffic_light_lookup
* traffic lights to stop line lookup fix
* Merge pull request `#621 <https://github.com/tier4/scenario_simulator_v2/issues/621>`_ from tier4/fix/empty_trafic_light
* publish empty message
* change topic name
* Merge pull request `#618 <https://github.com/tier4/scenario_simulator_v2/issues/618>`_ from tier4/fix/remove_lanechange_route
* random test runner
* set withLaneChange parameter as false
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge pull request `#612 <https://github.com/tier4/scenario_simulator_v2/issues/612>`_ from tier4/feature/remove_newton_method_from_get_s_value
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* use autoscale option
* add getNearbyLaneletIds function
* consider bounding box if possible
* remove default argument
* modify test cases
* add bounding box to the argument
* remove debug line
* enable consider edge case (tx and ty sometimes inf)
* simplify get s value algorithum
* add matchToLane function
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Merge branch 'feature/lane_matching' into feature/remove_newton_method_from_get_s_value
* remove newton method
* return boost::none when the value under 0 or over 1
* Revert "Merge pull request `#603 <https://github.com/tier4/scenario_simulator_v2/issues/603>`_ from tier4/fix/get_s_value"
* fix compile error
* add matchToLane function
* add lanelet2_matching package to the external directory
* Merge pull request `#609 <https://github.com/tier4/scenario_simulator_v2/issues/609>`_ from tier4/feature/load_plugin_in_spawn_api
* put values into TODO
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, dai0912th, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge pull request `#603 <https://github.com/tier4/scenario_simulator_v2/issues/603>`_ from tier4/fix/get_s_value
* apply reformat
* use size_t in catmull rom spline class
* use size_t in Hermite Curve class
* fix torelance in test cases
* use multiple initial value
* remove torelance check
* modify parameters
* fix logics if the s value unders 0 and overs 1
* Merge pull request `#597 <https://github.com/tier4/scenario_simulator_v2/issues/597>`_ from tier4/refactor/traffic_simulator/spawning
* Fix function name to lowerCamelCase from snake_case
* Lipsticks
* Replace flag `is_ego` to string typed plugin name
* Update predefined plugin names to use construnst on first use idiom
* Merge branch 'master' into feature/interpreter/catalog
* Add constexpr variable `default_behavior` as entity types member
* Cleanup `API::spawn` for Misc type entity
* Cleanup `API::spawn` for Pedestrian type entity
* Cleanup `API::spawn` for Vehicle type entity
* Update `API::spawn` argument order
* Remove meaningless argument `is_ego` from some `spawn` overloads
* Update `API::spawn` to not to apply `setEntityStatus` to rest arguments
* Update `AddEntityAction::operator ()` to use `TeleportAction::teleport`
* fix typo
* fix calcuration method of normal vector
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: Masaya Kataoka, MasayaKataoka, Yutaro Matsuura, kyabe2718, yamacir-kit

0.5.6 (2021-10-28)
------------------
* Merge pull request `#586 <https://github.com/tier4/scenario_simulator_v2/issues/586>`_ from tier4/fix/get_longitudinal_distance
* fix way of calculating longitudinal distance
* Merge pull request `#585 <https://github.com/tier4/scenario_simulator_v2/issues/585>`_ from tier4/feature/get_nearby_lanelet
* Merge pull request `#584 <https://github.com/tier4/scenario_simulator_v2/issues/584>`_ from tier4/fix/get_closet_lanelet_id
* fix typo
* add s in end
* remove unused orientation
* enable get nearby lanelet
* fix typo
* enable pass plugin name via constructor
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#573 <https://github.com/tier4/scenario_simulator_v2/issues/573>`_ from tier4/feature/behavior_debug_marker
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/galactic_docker_image
* Merge pull request `#575 <https://github.com/tier4/scenario_simulator_v2/issues/575>`_ from tier4/fix/typo
* fix compile errors
* fix typo detected from https://github.com/tier4/scenario_simulator_v2/runs/3923309766?check_suite_focus=true
* rename function
* set initial value in entity constructor
* enable get debug marker from blackboard
* add debug marker setter/getter
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#570 <https://github.com/tier4/scenario_simulator_v2/issues/570>`_ from tier4/feature/cleanup_logger
* change message type name
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* use const & in getCurrentAction function
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* enable publish debug marker
* add appenDebugMarker function
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge pull request `#568 <https://github.com/tier4/scenario_simulator_v2/issues/568>`_ from tier4/feature/clanup_macro_and_blackboard
* remove transition step from setup logger function
* enable pass logger
* sort lines asending
* remove blackboard and modify macro
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#566 <https://github.com/tier4/scenario_simulator_v2/issues/566>`_ from tier4/feature/behavior_plugin
* link stdc++fs
* comment out unused test cases
* fix depends and LICENSE
* modify install line
* add boost to the depends
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* apply reformat
* remove destructor
* use destructor
* add onDespawn function
* remove debug commands
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* remove constructor and add configure function
* enable pass compile
* add debug lines
* fix plugin macro
* enable load plugin
* update config directory
* enable pass compile errors
* remove const
* add BehaviorTreePlugin class
* use base class
* add DEFINE_GETTER_SETTER macro
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* add setter functions
* enable pass compile in traffic_simulator
* define setter/getter
* use shared ptr
* add base class
* add BlackBoard class
* change include path
* change include guard
* move behavior source codes from traffic_simulator to behavior_tree_plugin
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------
* Merge pull request `#562 <https://github.com/tier4/scenario_simulator_v2/issues/562>`_ from tier4/fix/depends_in_rviz
* fix rviz path and package dependency
* Contributors: Masaya Kataoka, MasayaKataoka

0.5.4 (2021-10-13)
------------------
* Merge pull request `#557 <https://github.com/tier4/scenario_simulator_v2/issues/557>`_ from tier4/revert/pr_544
* Revert "Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status"
* Merge pull request `#554 <https://github.com/tier4/scenario_simulator_v2/issues/554>`_ from tier4/feature/autoware/upper-bound-velocity
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Fix Autoware's default upper bound speed to double max from zero
* Add new member function `setUpperBoundSpeed`
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------
* Merge pull request `#549 <https://github.com/tier4/scenario_simulator_v2/issues/549>`_ from tier4/fix/traffic_simulator/autoware
* Fix `EgoEntity::setStatus` to call `VehicleEntity::setStatus`
* Lipsticks
* Merge pull request `#548 <https://github.com/tier4/scenario_simulator_v2/issues/548>`_ from prybicki/patch-6
* Fix SIGABRT due to accessing uninitialized optional
* Contributors: Peter Rybicki, Tatsuya Yamasaki, yamacir-kit

0.5.2 (2021-10-06)
------------------
* Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status
* initialize standstill duration for each entity
* apply reformat
* remove boost::none check
* add spawnEntity function
* remove boost::none from getStandstillDuration function
* remove boost none in each metrics
* enable check entity exists
* move rviz file and configure depends
* add API::
* add spawn function
* remove spawn function without status
* remove unused depend
* use template
* use API::setEntityStatus function
* enable pass compile
* add doxygen comments
* add name argument
* add comment
* remove unused bool return value
* remove boost::none status in traffic_simulator
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#536 <https://github.com/tier4/scenario_simulator_v2/issues/536>`_ from tier4/fix/get_longitudinal_distance
* check target entity is assing to the lane or not
* udpate Release note
* add bool EntityManager::laneMatchingSucceed(const std::string & name)
* Merge pull request `#533 <https://github.com/tier4/scenario_simulator_v2/issues/533>`_ from tier4/feature/interpreter/distance-condition
* Update `getLongitudinalDistance` to support overload for `LaneletPose`
* Merge pull request `#530 <https://github.com/tier4/scenario_simulator_v2/issues/530>`_ from RobotecAI/traffic_lights
* Typos fix
* Clang formatting and conversions test for traffic light
* ZMQ api for traffic lights
* Traffic lights wip
* Merge branch 'master' into fix/clean_directory_behavior
* Merge branch 'master' into rename_AA_launch_package
* Merge pull request `#491 <https://github.com/tier4/scenario_simulator_v2/issues/491>`_ from tier4/feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/interpreter/add-entity-action
* Merge pull request `#511 <https://github.com/tier4/scenario_simulator_v2/issues/511>`_ from tier4/feature/metrics_get_jerk_from_autoware
* trivially fix
* EntityManager has a node as rclcpp::node_interfaces::NodeTopicInterface to erase its type
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge remote-tracking branch 'origin/master' into feature/interpreter/distance-condition
* Merge pull request `#521 <https://github.com/tier4/scenario_simulator_v2/issues/521>`_ from tier4/feature/collision_metric
* update document and fix typo
* Merge pull request `#520 <https://github.com/tier4/scenario_simulator_v2/issues/520>`_ from tier4/feature/standstill_metric
* add test case
* enable specify targets
* enable throw spec violation
* modify cmakelist.txt
* Merge branch 'feature/standstill_metric' of https://github.com/tier4/scenario_simulator_v2 into feature/collision_metric
* add source
* add standstill duration scenario
* add standstill metric
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Merge pull request `#512 <https://github.com/tier4/scenario_simulator_v2/issues/512>`_ from tier4/feature/test_entity
* apply reformat
* add acquire position test cases
* add a subscription to get jerk
* add test case for set status and update timestamp
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Cleanup member function `EgoEntity::getCurrentAction`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Support new UserDefinedValueCondition `<ENTITY-NAME>.currentState`
* Support new member function `API::getCurrentAction`
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Jaroszek, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge pull request `#507 <https://github.com/tier4/scenario_simulator_v2/issues/507>`_ from tier4/feature/add_scenario
* update lane assing logic for pedestrian
* split function
* apply reformat
* enable get lanelet pose while walk straight action
* Merge pull request `#505 <https://github.com/tier4/scenario_simulator_v2/issues/505>`_ from tier4/feature/test_helper
* modify line
* add test case for lidar sensor
* add test case for constructing action status
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* gix some typo
* remove dists
* fix some typo
* fix filename
* use foo/bar/baz
* fix typo of Bounding
* fix typo of polynomial
* fix typo of cache
* fix typo of Valuet
* fix compile error
* change dist to distance
* fix typo of tolerance
* add test case for helper function
* Merge pull request `#501 <https://github.com/tier4/scenario_simulator_v2/issues/501>`_ from tier4/feature/add_test_traffic_light
* Merge pull request `#500 <https://github.com/tier4/scenario_simulator_v2/issues/500>`_ from tier4/fix/offset_calculation
* add test cases for >> operator
* add ss = std::stringstream(); lines
* add test case for operator <<
* Merge branch 'master' into fix/offset_calculation
* add // LCOV_EXCL_LINE
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* remove std::hypot(x,y,z)
* Merge pull request `#458 <https://github.com/tier4/scenario_simulator_v2/issues/458>`_ from Utaro-M/add-goalpose
* add subtraction
* Merge pull request `#486 <https://github.com/tier4/scenario_simulator_v2/issues/486>`_ from prybicki/patch-5
* fix toMapPose function
* add test cases
* add operator override
* add getSize function
* Fix bad formatting
* add linear algebra.cpp
* Merge pull request `#498 <https://github.com/tier4/scenario_simulator_v2/issues/498>`_ from tier4/feature/remove_unused_codes_in_entity
* remove pedestrian_parameters.hpp and vehicle_parameters.hpp
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_unused_codes_in_entity
* fix typo
* Merge branch 'master' into add-goalpose
* Merge pull request `#492 <https://github.com/tier4/scenario_simulator_v2/issues/492>`_ from tier4/feature/add_traffic_light_test
* use static cast in std::accumulate function
* use reference
* remove unused function
* remove unused function
* add test cases for getArrow function
* add test line
* remove std::accumulate because of overflow
* fix update logic in traffic light phase class
* add update line in test case
* add setColorPhase test cases
* fix error
* fix values
* add getColorAndArrowPosition test cases
* add test cases for getArrowPosition
* add expect macro
* add set arrow function
* use foreach
* add test cases for setColor function
* remove test case file
* add new test case source
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge pull request `#489 <https://github.com/tier4/scenario_simulator_v2/issues/489>`_ from tier4/feature/test_traffic_light
* use std::find instead of std::count_if
* add test case for getIDs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* add new test case file
* Merge pull request `#487 <https://github.com/tier4/scenario_simulator_v2/issues/487>`_ from tier4/feature/get_longituninal_distance_behind
* add test case for arrow and NONE type
* enable get distance from behind entity
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Merge pull request `#485 <https://github.com/tier4/scenario_simulator_v2/issues/485>`_ from tier4/feature/test_simulation_interface
* add test case for makeLampState function
* Update `EgoEntity` to default construct `Autoware` if `launch_autoware == false`
* Merge branch 'master' into add-goalpose
* fix typo
* Set name in the proto request for non-ego vehicles
* Support new option `initialize_duration`
* Update class `EgoEntity` to don't instantiate class `Autoware` if `launch_autoware == false`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Rename option `architecture-type` to `architecture_type`
* Remove unreachable return in API::spawn (`#480 <https://github.com/tier4/scenario_simulator_v2/issues/480>`_)
* Set correct entity names in proto messages (`#481 <https://github.com/tier4/scenario_simulator_v2/issues/481>`_)
* Feature/request acuire position in world coordinate (`#439 <https://github.com/tier4/scenario_simulator_v2/issues/439>`_)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* add goalpose arrow
* add getGoalposes()
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Peter Rybicki, Piotr Rybicki, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Fix/offset calculation in lane coordinte (`#476 <https://github.com/tier4/scenario_simulator_v2/issues/476>`_)
* Merge pull request `#475 <https://github.com/tier4/scenario_simulator_v2/issues/475>`_ from tier4/feature/add_math_test
* Merge pull request `#437 <https://github.com/tier4/scenario_simulator_v2/issues/437>`_ from RobotecAI/issue/AJD-237-remove_autoware_compilation_flag
* add getSvalue test
* modify test cases in getTrajectory
* add test case for get trajectory function
* add test case for search backwards
* fix compile errors
* remove using
* Merge remote-tracking branch 'origin/master' into fix/interpreter/misc
* Feature/metrics test (`#469 <https://github.com/tier4/scenario_simulator_v2/issues/469>`_)
* Feature/remove unused constructor (`#465 <https://github.com/tier4/scenario_simulator_v2/issues/465>`_)
* Merge pull request `#464 <https://github.com/tier4/scenario_simulator_v2/issues/464>`_ from tier4/feature/add_math_test
* fix build and formatting after rebase
* review changes
* apply clang-format
* cleanup
* AAP acceleration fix
* make Autoware switch based on autoware_type parameter
* AAP builds
* first version that builds without flag and works with AA on autoware-simple scenario
* move Autoware differences from ego_entity to concealer
* WIP: move Autoware differences from ego_entity to concealer
* remove unused temporal value
* add collision test case
* add subdirectory
* enable matrix test
* add test cases for solve quadratic equation
* fix quadratic function ans
* add quadratic function case
* add LinerFunction case
* add range test
* add LinerFunction
* update uuid test case
* change test name
* remove math.cpp
* add uuid case
* modify testcase name
* add bounding box test case
* add subdirectory
* add catmull rom spline test cases
* remove unused code
* remove unused test case
* add fixture class
* add hermite curve test cases
* change test case name
* split test cases
* add test cases for getCollisionPositionIn2D function
* add test case for auto scale
* initialize current_time\_ with negative value so setTargetSpped and setTargetVelocity are deterministic (`#462 <https://github.com/tier4/scenario_simulator_v2/issues/462>`_)
* Feature/move backward action (`#461 <https://github.com/tier4/scenario_simulator_v2/issues/461>`_)
* Merge pull request `#457 <https://github.com/tier4/scenario_simulator_v2/issues/457>`_ from tier4/feature/math_test
* fix typo
* add // LCOV_EXCL_LINE
* chceck
* add test cases
* add  // LCOV_EXCL_LINE
* add  // LCOV_EXCL_LINE
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* add // LCOV_EXCL_LINE
* add // LCOV_EXCL_LINE
* remove unused message
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* add test case in maximum curventure
* add get2DMinMaxCurventureValue function
* adding test cases
* remove unused message
* add error test
* modify error message
* remove unused constractor
* remove unused line
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge pull request `#451 <https://github.com/tier4/scenario_simulator_v2/issues/451>`_ from tier4/feature/out-of-range-metric
* Merge pull request `#452 <https://github.com/tier4/scenario_simulator_v2/issues/452>`_ from tier4/fix/stop_at_crossing_entity_behavior
* remove unused line
* remove unused line
* add test scenario
* fix problems in stop_at_crossing_entity action
* Merge pull request `#450 <https://github.com/tier4/scenario_simulator_v2/issues/450>`_ from tier4/fix/phase_control
* fix
* add OutOfRangeMetric when vehicle is spawned
* enable filter result
* remove const
* fix problems in TrafficLightManager::update() function
* Merge pull request `#448 <https://github.com/tier4/scenario_simulator_v2/issues/448>`_ from tier4/feature/add_cpp_scenarios
* add call thread::join in destructor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge pull request `#446 <https://github.com/tier4/scenario_simulator_v2/issues/446>`_ from tier4/feature/out-of-range-metric
* fix exception message
* add out_of_range_metric.cpp to CMakeLists.txt
* add OutOfRangeMetric
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* add linear velocity to MomentaryStopMetric
* add metrics scenario
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge pull request `#432 <https://github.com/tier4/scenario_simulator_v2/issues/432>`_ from tier4/fix/suppress_warnings
* remove boost none in getFrontEntityName logic
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/suppress_warnings
* Merge pull request `#440 <https://github.com/tier4/scenario_simulator_v2/issues/440>`_ from tier4/fix/follow_front_entity_behavior
* leave @note line
* use const &
* use const &
* remove unused temporal value
* fix torerance
* remove debug lines
* fix cubic function
* enable check collision
* Merge branch 'master' into namespace
* change default value
* add close_start_end option
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#434 <https://github.com/tier4/scenario_simulator_v2/issues/434>`_ from tier4/feature/remove_get_s_valueIn_route
* remove unused functions
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#433 <https://github.com/tier4/scenario_simulator_v2/issues/433>`_ from tier4/feature/yeild_to_merging_entity
* apply reformat
* remove debug lines
* add getEntityStatus function
* apply reformat
* get polygon to the other status
* fix to old version get longitudinal distance function
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/lane_change_npc_distance_in_lane_coordinate
* fix no return
* fix unused
* fix uninitialized
* fix rclcpp::Duration deprecated
* add Werror
* change quadratic function
* Merge remote-tracking branch 'origin/master' into namespace
* check error
* fix s value
* check tx and ty value
* fix othre conditions
* fix collision solver for cubic function
* enable calculate distance to front entity by spline
* enable get distance between polygon
* remove unused result
* remove debug lines
* use getConflictingEntityStatusOnRoute in getConflictingEntityStatus function
* enable get conflicting vehicle entity in foundConflictingEntity function
* remove unused function
* add std::vector<std::int64_t> HdMapUtils::getConflictingLaneIds
* Merge pull request `#429 <https://github.com/tier4/scenario_simulator_v2/issues/429>`_ from tier4/feature/add_cpp_scenarios
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* set initial value for current_s\_
* fix argument name
* fix compile errors
* Merge branch 'fix/add_include_crosswalk_option' of github.com:tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* Contributors: Hiroki OTA, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------
* Merge pull request `#425 <https://github.com/tier4/scenario_simulator_v2/issues/425>`_ from tier4/fix/add_include_crosswalk_option
* Merge pull request `#408 <https://github.com/tier4/scenario_simulator_v2/issues/408>`_ from RobotecAI/issue/AJD-239-non_hardcoded_map
* apply reformat
* use calculateEntityStatusUpdated function
* kashiwanoha_map package map files added
* Contributors: Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki

0.4.1 (2021-07-30)
------------------
* Merge pull request `#421 <https://github.com/tier4/scenario_simulator_v2/issues/421>`_ from tier4/fix/npc_target_speed_in_follow_front_entity_action
* fix velocity planner logic
* apply reformat
* Merge pull request `#419 <https://github.com/tier4/scenario_simulator_v2/issues/419>`_ from tier4/feature/rename_moc_to_mock
* enable set waypoints
* fix login in follow front entity action
* remove launch install line
* remove moc directory
* move to mock pacakge
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Contributors: Masaya Kataoka, yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#407 <https://github.com/tier4/scenario_simulator_v2/issues/407>`_ from tier4/feature/galactic_support
* apply reformat
* fix usage of declare_parameter function
* check boost::none in update function
* remove undeclare line
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Move flag 'autoware_initialized' into class 'Autoware'
* Update EgoEntity to occupy one Autoware each
* Remove debug codes from EgoEntity
* Remove member function 'getMapPath'
* Add member function 'get*MapFile' to struct Configuration
* Update class Configuration to assert .osm file existence
* Update class Configuration to assert given map_path
* Add data member 'rviz_config_path' to struct Configuration
* Update EgoEntity's constructor to receive Configuration
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* Lipsticks
* Remove some unused data members
* Merge pull request `#403 <https://github.com/tier4/scenario_simulator_v2/issues/403>`_ from tier4/feature/target_speed_planner
* Update EntityManager's constructor to receive Configuration
* add const
* enable use target speed planner in pedestrian entity
* Fix lanelet projector
* enable use target_speed_planner in vehicle entity
* Move class 'Configuration' into new header 'configuration.hpp'
* Update MetricsManager to receive boost::filesystem::path
* Cleanup
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* add new file
* Update HDMapUtils's constructor to receive boost::filesystem::path
* remove value
* Merge pull request `#400 <https://github.com/tier4/scenario_simulator_v2/issues/400>`_ from tier4/feature/remove_unused_member_value_in_entity
* Merge pull request `#401 <https://github.com/tier4/scenario_simulator_v2/issues/401>`_ from tier4/fix/typo
* Remove some duplicated API's data members
* remove getCurrentAction and replace to tickOnce(current_time, step_time);
* fix typo of calculate
* change return type to void
* remove action status from pedestrian entity
* Add new struct 'Configuration' for class 'API'
* fix compile error
* remove from member
* remove action_status from vehicle entity
* Merge remote-tracking branch 'origin/master' into fix/interpreter/acquire-position-action
* Merge pull request `#390 <https://github.com/tier4/scenario_simulator_v2/issues/390>`_ from tier4/feature/interpreter/traffic-signal-controller-condition
* Fix syntax Event and ManeuverGroup to be able to restart elements
* Update TrafficLightManager to store TrafficLight objects directly
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Update enumeration 'Arrow'
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge pull request `#386 <https://github.com/tier4/scenario_simulator_v2/issues/386>`_ from tier4/feature/interpreter/misc-object
* Fix rviz config paths
* Merge pull request `#387 <https://github.com/tier4/scenario_simulator_v2/issues/387>`_ from tier4/fix/delete_whole_route_when_empty
* apply reformat
* fix route planner logic
* Merge remote-tracking branch 'origin/master' into feature/interpreter/test-scenario
* Merge pull request `#384 <https://github.com/tier4/scenario_simulator_v2/issues/384>`_ from tier4/feature/interpreter/assign-route-action-with-world-position
* Merge remote-tracking branch 'origin/master' into feature/interpreter/assign-route-action-with-world-position
* Merge pull request `#328 <https://github.com/tier4/scenario_simulator_v2/issues/328>`_ from RobotecAI/pjaroszek/map_and_planning
* Lipsticks
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* clang formatting
* enable send SpawnMiscObjectEntityRequest to the sensor simulator
* update mock
* add despawn line
* specify entity type
* fix debug message
* fix compile errors
* adapt formatting
* rebase adjustments
* add constructor
* add MiscObjectParameters message type
* Merge branch 'master' into traffic_signal_actions
* build with AUTOWARE_AUTO flag defined instead of AUTOWARE_ARCHITECTURE_PROPOSAL
* Merge pull request `#379 <https://github.com/tier4/scenario_simulator_v2/issues/379>`_ from tier4/fix/get_waypoints_error_message
* fix debug line
* apply reformat
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/get_waypoints_error_message
* chenge error message
* Merge pull request `#378 <https://github.com/tier4/scenario_simulator_v2/issues/378>`_ from tier4/feature/ego-entity/acuquire-position-action
* Update EgoEntity to be able to request AcquirePositionAction multiple times
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.2.0 (2021-06-24)
------------------
* Merge pull request `#372 <https://github.com/tier4/scenario_simulator_v2/issues/372>`_ from tier4/fix/lane_change_parameter
* add New lines at the end of the file
* apply reformat
* input optional arguments
* update lane change logic
* move getRelativePose function to the math directory
* enable set maximum_curvature_threshold parameter
* Merge pull request `#357 <https://github.com/tier4/scenario_simulator_v2/issues/357>`_ from tier4/feature/send_ego_command
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* enable use getEntityStatusBeforeUpdate function in EntityManager class
* add getEntityStatusBeforeUpdate() function
* fix problems while getting ego status
* remove warnings and add getEgoName function to the API class
* add getEgoName function to the entity manager class
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge pull request `#363 <https://github.com/tier4/scenario_simulator_v2/issues/363>`_ from tier4/fix/throw_errors_when_set_target_speed_after_ego_starts
* apply clanf-format
* enable throw semantic error for setTargetSpeed and setEntityStatus after starting simulation
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'feature/interpreter/context' of github.com:tier4/scenario_simulator_v2 into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge branch 'master' into relative_target_speed
* Revert "add RelativeTargetSpeed support to interpreter"
* add RelativeTargetSpeed support to interpreter
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge pull request `#355 <https://github.com/tier4/scenario_simulator_v2/issues/355>`_ from tier4/feature/get_vehicle_cmd
* add getVehicleCommand class to the API class
* add const autoware_vehicle_msgs::msg::VehicleCommand getVehicleCommand(); function to the EntityManager class
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/get_vehicle_cmd
* add getVehicleCommand function to the ego entity class
* Merge pull request `#354 <https://github.com/tier4/scenario_simulator_v2/issues/354>`_ from tier4/fix/typos-misc
* Apply clang-format
* Fix typos in docs / mock / simulation/ test_runner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into doc/update_image
* Merge pull request `#351 <https://github.com/tier4/scenario_simulator_v2/issues/351>`_ from tier4/fix/traffic-simulator/simulation-model-2
* Replace some identifiers spellcheck reported as issue
* Replace temporary excptions
* Update EgoEntity::setTargetSpeed to consider parameter 'vehicle_model_type'
* Support simulation model 'SimModelIdealSteer'
* Update EgoEntity to use model specified by parameter 'vehicle_model_type'
* Update EgoEntity to read parameter 'vehicle_model_type'
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/replay.launch
* Merge pull request `#345 <https://github.com/tier4/scenario_simulator_v2/issues/345>`_ from tier4/fix/traffic-simulator/simulation-model
* Fix control inputs
* Add some debug prints
* Update EgoEntity's simulation model to use SimModelTimeDelaySteerAccel
* Merge pull request `#343 <https://github.com/tier4/scenario_simulator_v2/issues/343>`_ from tier4/fix/traffic-simulator/vehicle-parameter
* Fix API to pass step-time to EgoEntity's constructor
* Add debug prints
* Remove debug print
* Update launch file to receive LaunchContext
* Merge pull request `#338 <https://github.com/tier4/scenario_simulator_v2/issues/338>`_ from tier4/feature/interpreter/vehicle-description
* Update EgoEntity to use precise simulation model parameters
* Update scenario_test_runner.launch.py to load Autoware parameter
* Merge pull request `#336 <https://github.com/tier4/scenario_simulator_v2/issues/336>`_ from tier4/feature/speed_up_npc_logic
* add meanings  of sampling resolution
* fixd problems described in review
* remove verbose true
* remove unused lines
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* apply reformat
* Fix typos and grammars in docs and comments
* apply reformat
* add LaneletLengthChache
* filter other entity status by cartesian distance
* comment out multithread section
* commend out stop watch
* calculate waypoints only one time in follow lane action
* add center points cache
* fix typo
* apply reformat
* use future and async launch
* remove warning
* add parallel util
* use openmp
* add omp to the depends
* fix typo
* update branch
* apply reformat
* enable cache length
* apply reformat
* add route cache
* Merge pull request `#316 <https://github.com/tier4/scenario_simulator_v2/issues/316>`_ from tier4/fix/hold_stream
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into fix/hold_stream
* apply reformat
* hold ostrem in manager class
* change default value
* apply reformat
* add file_output_every_frame option (default = false)
* Merge pull request `#315 <https://github.com/tier4/scenario_simulator_v2/issues/315>`_ from tier4/feature/use_ros_clock
* enable use raw ros timestamp
* Merge pull request `#314 <https://github.com/tier4/scenario_simulator_v2/issues/314>`_ from tier4/fix/sensor_timestamp
* apply reformat
* Merge pull request `#306 <https://github.com/tier4/scenario_simulator_v2/issues/306>`_ from tier4/feature/use_common_exception
* Remove trailing semicolon from macro definition
* fix problems in https://github.com/tier4/scenario_simulator_v2/pull/306#discussion_r634055658
* fix typo
* add spec violation error
* remove BehaviorTree exception
* remove CALCURATION_ERROR
* remove SimulationClockError
* remove Execution Error
* remove some exception
* remove exception.hpp
* remove hdmap error
* use common exception
* remove traffic_simulator::SimulationRuntimeError
* modify macro
* Merge pull request `#304 <https://github.com/tier4/scenario_simulator_v2/issues/304>`_ from tier4/feature/synchronize_clock
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronize_clock
* enable send clock
* Merge pull request `#302 <https://github.com/tier4/scenario_simulator_v2/issues/302>`_ from tier4/feature/error-handling-2
* Merge pull request `#301 <https://github.com/tier4/scenario_simulator_v2/issues/301>`_ from tier4/feature/publish_clock
* add licence
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge pull request `#297 <https://github.com/tier4/scenario_simulator_v2/issues/297>`_ from tier4/feature/error-handling
* use from_seconds function
* enable publish clock
* add publisher
* Update Interpreter to destruct simulator on deactivation phase
* remove unused params
* add clock class to the api
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* add SimulationClock class
* Merge remote-tracking branch 'origin/master' into feature/error-handling
* Update EgoEntity to show current-time
* Contributors: Kazuki Miyahara, Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#295 <https://github.com/tier4/scenario_simulator_v2/issues/295>`_ from tier4/fix/python_format
  reformat by black
* reformat by black
* Merge pull request `#294 <https://github.com/tier4/scenario_simulator_v2/issues/294>`_ from tier4/feature/support-autoware.iv-0.11.2
  Feature/support autoware.iv 0.11.2
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* Update EgoEntity::getCurrentAction to return non-empty string
* remove flake8 check
* add new line for the block
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* Update scenario_test_runner.launch.py to receive sensor and vehicle model
* Merge pull request `#287 <https://github.com/tier4/scenario_simulator_v2/issues/287>`_ from tier4/feature/remove-dummy-perception-publisher
  Feature/remove dummy perception publisher
* Rename package 'awapi_accessor' to 'concealer'
* Update Autoware::ready to rethrow exception if there is thrown exception
* Merge pull request `#281 <https://github.com/tier4/scenario_simulator_v2/issues/281>`_ from tier4/feature/asynchronous-autoware-initialization
  Feature/asynchronous autoware initialization
* Update Storyboard to call engage if Autoware is ready (= WaitingForEngage)
* Lipsticks
* Cleanup EntityManager
* Update EntityManager to hold shared_ptrs as const
* Sort member functions of EntityManager
* Add member function EgoEntity::ready
* Update Autoware::engage to be synchronous
* Sort member functions of EgoEntity
* Add new virtual function EntityBase::getEntityTypename
* Update EntityBase to define default implementation of requestWalkStraight
* Sort member functions
* Generalize member function 'Autoware::plan's argument
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/update_contact_information
* Rename member function 'drive' to 'plan'
* Lipsticks
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-0.11.1
* Merge pull request `#276 <https://github.com/tier4/scenario_simulator_v2/issues/276>`_ from tier4/feature/autoware-high-level-api
  Feature/autoware high level api
* Move core procedures of requestAcquirePosition into class 'Autoware'
* Merge pull request `#277 <https://github.com/tier4/scenario_simulator_v2/issues/277>`_ from tier4/doc/docker
  Doc/docker
* Fix some EntityBase's member functions to be virtual
* Fix some EntityBase class member functions
* Update class 'Autoware' to update vehicle informations continuously
* Update EgoEntity to hold class 'Autoware' as non-pointer
* add virtual and override
* add debug lines
* remove warnings
* Update promise to be non-pointer data member
* Update EgoEntity to be uncopyable
* Lipsticks
* Move Autoware process control into class 'Autoware'
* Move Autoware initialization into class 'Autoware' from 'EgoEntity'
* Move member function 'updateAutoware' into AutowareAPI
* Move EgoEntity's member functions 'waitForAutowareStateToBe...' into awapi
* Merge branch 'feature/support-autoware.iv-0.11.1' into feature/autoware-high-level-api
* Merge pull request `#274 <https://github.com/tier4/scenario_simulator_v2/issues/274>`_ from tier4/refactor/cleanup-ego-entity
  Refactor/cleanup ego entity
* Update entity_base::setDriverModel to be virtual
* Fix bug
* Merge branch 'refactor/cleanup-ego-entity' into feature/autoware-high-level-api
* Merge remote-tracking branch 'origin/master' into refactor/cleanup-ego-entity
* Merge pull request `#275 <https://github.com/tier4/scenario_simulator_v2/issues/275>`_ from tier4/feature/init_duration
  Feature/init duration
* return null obstacle when current time while initializing
* fix problems in getting obstacle
* add debug lines
* Rename 'awapi_accessor' to 'autoware'
* enable visualize obstacle
* Lipsticks
* apply reformat
* update mock
* Rename autoware_api::Accessor to awapi::Autoware
* remove at function when I emplace value
* remove boost::any
* Update EgoEntity::launchAutoware to receive map paths
* Convert launch_autoware to member function from closure
* Move EgoEntity::initializeAutoware into ego_entity.cpp
* Convert 'get_parameter' to template function from generic lambda
* Move EgoEntity::requestAcquirePosition into ego_entity.cpp
* Move EgoEntity's destructor into ego_entity.cpp
* enable spawn entity
* Move EgoEntity::EgoEntity into ego_entity.cpp
* Cleanup comments
* remove debug lines
* add init_duration
* Update awapi_accessor to publish '/localization/twist'
* Update EgoEntity to launch Autoware via autoware_launch
* Merge branch 'master' into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#265 <https://github.com/tier4/scenario_simulator_v2/issues/265>`_ from tier4/feature/interpolate_two_center_points
  interpolate center points if the center points are only two points
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpolate_two_center_points
* interpolate center points if the center points are only two points
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#263 <https://github.com/tier4/scenario_simulator_v2/issues/263>`_ from tier4/feature/traffic-signal-sensor
  Feature/traffic signal sensor
* Merge pull request `#264 <https://github.com/tier4/scenario_simulator_v2/issues/264>`_ from tier4/revert/interpolate_two_points
  Revert "enable interpolate two points"
* Revert "enable interpolate two points"
  This reverts commit 7b08f1d0de38e9b31e1d066d5c6ed7faec6758bd.
* enable interpolate two points
* Update traffic signals topic name to use AWAPI
* Update TrafficLightArrow to support conversion to 'LampState' type
* Lipsticks
* Update TrafficLightColor to support conversion to 'LampState'
* Add stream input/output operator to TrafficLight(Arrow|Color)
* Lipsticks
* Rename 'PhaseLength' to 'PhaseDuration'
* Unify some member function definitions into a macro
* Update EntityManager to pass traffic light states publisher to TrafficLightManager
* Merge https://github.com/tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Lipsticks
* Merge pull request `#262 <https://github.com/tier4/scenario_simulator_v2/issues/262>`_ from tier4/feature/interpreter/traffic-signal-controller-2
  Feature/interpreter/traffic signal controller 2
* Add local macro 'RENAME'
* Lipsticks
* Merge branch 'master' into fix/misc-problems
* Merge pull request `#238 <https://github.com/tier4/scenario_simulator_v2/issues/238>`_ from tier4/feature/interpreter/vehicle/base_link-offset
  Remove member function `API::spawn` receives XML strings.
* Remove member function 'API::spawn' receives catalog XML
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* fix launch file
* update namespace
* use clang_format
* apply reformat
* Merge https://github.com/tier4/scenario_simulator.auto into feature/rename_packages
* modify include gurard
* rename simulation_api package
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit
