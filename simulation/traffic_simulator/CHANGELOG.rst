^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* enable chache length
* apply reformat
* add route chache
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
