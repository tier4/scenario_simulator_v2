^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concealer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.7 (2022-11-17)
------------------
* refactor(preprocessor): modify struct name
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* Merge branch 'master' into fix/interpreter/custom_command_action
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#848 <https://github.com/tier4/scenario_simulator_v2/issues/848>`_ from RobotecAI/fix/service-request-until-success
* Add TODO comments
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* clang format
* removed logs
* log
* single engage call
* mode logs
* test logs
* debug log
* Merge branch 'master' into feature/interpreter/priority
* Revert "increased set velocity attempts count"
* fix
* increased set velocity attempts count
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* slight reformat; added exception when service does not succeed
* Merge pull request `#875 <https://github.com/tier4/scenario_simulator_v2/issues/875>`_ from tier4/feature/concealer/acceleration
* Merge remote-tracking branch 'origin/master' into feature/concealer/acceleration
* Merge pull request `#823 <https://github.com/tier4/scenario_simulator_v2/issues/823>`_ from tier4/feature/start_npc_logic_api
* review application
* Update `AutowareUniverse` to publish current acceleration
* Fix `AutowareUniverse::(engageable|engaged)` to check Autoware's exception
* Lipsticks
* Rename member function `Autoware::ready` to `Autoware::engaged`
* license information added
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* code review suggestions application
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into fix/simple_sensor_simulator/fast_occupancy_grid
* Remove unused member function `AutowareUniverse::isReady`
* Update `Interpreter` to set `Configuration::initialize_duration` to zero
* templated service interface implemented; service request until success
* Update `Interpreter` to start non-ego entities at Autoware reaches `DRIVING` state.
* Merge remote-tracking branch 'origin/refactor/interpreter/scope' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge pull request `#794 <https://github.com/tier4/scenario_simulator_v2/issues/794>`_ from tier4/fix/interpreter/transition_assertion
* Fix typo
* remove initialize_duration time limit from waitForAutowareStateToBeDriving
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/value_constraint
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/scope
* Merge pull request `#841 <https://github.com/tier4/scenario_simulator_v2/issues/841>`_ from tier4/feature/cooperator
* Merge remote-tracking branch 'origin/master' into feature/cooperator
* Merge remote-tracking branch 'origin/master' into feature/use_github_registry
* Merge pull request `#842 <https://github.com/tier4/scenario_simulator_v2/issues/842>`_ from RobotecAI/fix/concealer-dangling-reference
* Move some function implementation into `autoware_universe.cpp`
* clang format
* Revert some changes
* Update `RTC` to push request into `cooperate_queue` instead of call it
* Cleanup
* Move RTC features into struct `RTC` from `AutowareUniverse`
* Rename enumeration `Cooperator` to `Cooperator::Is`
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* build fixes
* concealer getters return value instead of const reference
* Revert "path subscribtion macros enrolled and returned by value"
* path subscribtion macros enrolled and returned by value
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge pull request `#818 <https://github.com/tier4/scenario_simulator_v2/issues/818>`_ from tier4/feature/autoware/request-to-cooperate
* Rename `CooperatePolicy` to `Cooperator`
* Cleanup
* Add new experimental Controller property `cooperatePolicy`
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Fix subscription to be compilable with ROS2 Humble
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'feature/get_distance_to_lane_bound' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* fix trivial
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' into fix/interpreter/transition_assertion
* Update `AutowareUniverse` to subscribe `/api/external/get/rtc_status`
* TransitionAssertion measures the duration from the start
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#813 <https://github.com/tier4/scenario_simulator_v2/issues/813>`_ from tier4/fix/boost_depend
* add boost to the depends of concealer and interpreter
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into fix/build-error-humble
* Merge pull request `#804 <https://github.com/tier4/scenario_simulator_v2/issues/804>`_ from tier4/feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* Update `AutowareUniverse::request##TYPE` to not to re-request service
* fix concealer subscription macro
* add compile definition for tf2 geometry msgs
* fix(concealer): modify build error in humble
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_lane_bound
* Merge pull request `#796 <https://github.com/tier4/scenario_simulator_v2/issues/796>`_ from tier4/refactor/concealer/virtual-functions
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Remove member function `API::engage` and `API::ready`
* Remove some member functions for Autoware.Universe from API
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/arrange_docs_and_fix_copyright
* Fix Licence
* Add virtual function `getTurnIndicatorsCommand` to class `Autoware`
* Add virtual function `getGearCommand` to class `Autoware`
* Merge pull request `#750 <https://github.com/tier4/scenario_simulator_v2/issues/750>`_ from tier4/fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge pull request `#792 <https://github.com/tier4/scenario_simulator_v2/issues/792>`_ from tier4/fix/autoware/reverse-gear
* Fix Autoware.Universe to accept `GearCommand` correctly
* feat(api): change set/engage to external api
* remove unnecessary file
* format
* trivial fix
* reformat
* make AutowareUniverse's subscribers and get##TYPE() non-blocking
* change the exit system of task_queue from future/promise to atomic bool
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Rename type aliases to match .Universe typenames
* Lipsticks
* Fix `concealer::AutowareUniverse` to publish received `GearCommand` as `GearReport`
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* format
* throw AutowareError from checkAutowareProcess
* add autoware->rethrow() in EgoEntity::onUpdate()
* Merge branch 'master' into feature/zmqpp_vendor
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/improve
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge pull request `#777 <https://github.com/tier4/scenario_simulator_v2/issues/777>`_ from tier4/feature/indicator_condition
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Refactor
* Rename **StateString to **StateName
* Fix linter error
* Implement TurnIndicatorsState as an UserDefinedValueCondition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge pull request `#760 <https://github.com/tier4/scenario_simulator_v2/issues/760>`_ from tier4/feature/emergency_state_for_fault_injection
* Refactor
* Add getEmergencyStateString function to EntityBase class
* refactor
* Use common::Error instead of SEMANTIC_ERROR
* Refactor
* Fix typo
* Add emergency state subscription to concealer::AutowareUniverse
* ament_clang_format
* fix typo
* check subprocess on the concealer::Autoware's thread
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* reformat
* revise unnecessary change
* fail if autoware_launch_file doesn't exist
* Contributors: Adam Krawczyk, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, wep21, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge pull request `#733 <https://github.com/tier4/scenario_simulator_v2/issues/733>`_ from tier4/feature/improve_ego_lane_matching
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* use dynamic_cast
* fix virtual specifier
* add getPath function
* apply reformat
* enable matching to on route lane
* add getRouteLanelets function
* fixes and cleanup
* ugly but working
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* fixes and cleanup
* ugly but working
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* Merge pull request `#704 <https://github.com/tier4/scenario_simulator_v2/issues/704>`_ from tier4/feature/autoware-external-api
  Feature/autoware external api
* Remove macro `CONCEALER_CURRENT_VALUE_OF`
* Remove header `undefine_macro.hpp`
* Add `CONCEALER\_` prefix to `concealer`'s macros
* Remove member function `AutowareUniverse::checkAutowareState`
* Rename member function `setUpperBoundSpeed` to `setVelocityLimit`
* Remove `HazardLightsReport` and `HazardLightsCommand` from dependency
* Remove member function `AutowareUniverse::setLaneChangeApproval`
* Remove member function `getVehicleStatus`
* Fix macro `CONCEALER_DEFINE_CLIENT` to check status code of response
* Add new macro `CONCEALER_INIT_CLIENT`
* Add new macro `CONCEALER_DEFINE_CLIENT`
* Add missing package dependency
* Replace engagement API with new ROS2 service version
* Remove macro `DEFINE_SUBSCRIPTION_WITH_OVERRIDE`
* Remove `tier4_system_msgs` from dependency
* Replace `AwapiAutowareStatus` with `autoware_auto_system_msgs::msg::AutowareState`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Remove class `AutowareAuto`
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Remove package `autoware_control_msgs` from dependency
* Remove package `autoware_vehicle_msgs` from dependency
* Replace `VehicleCommand` with `AckermannControlCommand` and `GearCommand`
* Remove package `autoware_perception_msgs`
* Replace `autoware_debug_msgs` with `tier4_debug_msgs`
* Remove some unused dependencies
* Remove class `AutowareArchitectureProposal`
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* merge fix/galactic_build
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into matsuura/feature/add-time-to-panel
* pull master
* merge master
* Merge tier4:master
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/get_driver_model_in_pedestrian
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change
* Merge pull request `#647 <https://github.com/tier4/scenario_simulator_v2/issues/647>`_ from tier4/tier4/universe
* Lipsticks
* Update `AutowareUniverse` to use message types `tier4\_*_msgs`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: Masaya Kataoka, MasayaKataoka, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Fix `waitForAutowareStateToBe*` to receive std::chrono::seconds
* Fix `TransitionAssertion` to stop if class `Autoware` down
* Fix `waitForAutowareStateToBe*` to call thunk at least one time.
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Add macro to switching Autoware.Auto or Autoware.Unvierse (DIRTY HACK)
* Remove `autoware_auto_msgs` from dependency
* Simplify `autoware_universe.hpp`
* Restore virtual function `Autoware::getVehicleCommand` for backward compatibility
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* Lipsticks
* detected object -> predicted object, and apply ament_clang_format
* Remove member function `getVehicleCommand` from Vehicle type entity
* fix concealer to execute without segmentation fault
* remove autoware_auto_msgs dependency
* check TODO(murooka), and remove them
* add covariance to odometry
* enable CurrentSteering publisher
* use autoware_auto_perception/planning_msgs
* remove unnecessary codes
* add some comments
* some changes to run psim with autoware_universe
* impelement autoware_universe
* set autoware_universe as aap
* add autoware_universe
* fix dependency
* Update some packages to use `tier4/autoware_auto_msgs`
* Contributors: MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_newton_method_from_get_s_value
* Merge pull request `#611 <https://github.com/tier4/scenario_simulator_v2/issues/611>`_ from tier4/fix/documentation
* Fix typo
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: kyabe2718

0.5.6 (2021-10-28)
------------------
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Contributors: MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------

0.5.4 (2021-10-13)
------------------
* Merge pull request `#554 <https://github.com/tier4/scenario_simulator_v2/issues/554>`_ from tier4/feature/autoware/upper-bound-velocity
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Fix Autoware's default upper bound speed to double max from zero
* Fix `setVehicleVelocity` to work in `Autoware::update`
* Add new member function `setUpperBoundSpeed`
* Lipsticks
* Contributors: Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------

0.5.2 (2021-10-06)
------------------
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Contributors: yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Update `EgoEntity` to default construct `Autoware` if `launch_autoware == false`
* Merge branch 'master' into add-goalpose
* Update class `TransitionAssertion` to use ROS parameter `initialize_duration`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Merge pull request `#437 <https://github.com/tier4/scenario_simulator_v2/issues/437>`_ from RobotecAI/issue/AJD-237-remove_autoware_compilation_flag
* minor changes: createUpdater -> resetTimerCallback and comment on sendSIGINT
* Revert "make the updater constant"
* make the updater constant
* fix build and formatting after rebase
* review changes
* warning fixes
* apply clang-format
* add .cpp files for autowares
* cleanup
* AAP acceleration fix
* pure virtual function call fix
* warining fixes
* make Autoware switch based on autoware_type parameter
* AAP builds
* first version that builds without flag and works with AA on autoware-simple scenario
* move Autoware differences from ego_entity to concealer
* WIP: move Autoware differences from ego_entity to concealer
* switch to AutowareAuto
* Merge pull request `#444 <https://github.com/tier4/scenario_simulator_v2/issues/444>`_ from tier4/feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Lipsticks
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge pull request `#432 <https://github.com/tier4/scenario_simulator_v2/issues/432>`_ from tier4/fix/suppress_warnings
* fix reorder
* Merge remote-tracking branch 'origin/master' into namespace
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* Update class Autoware to publish LocalizationPose for each frame
* Change topic name of PoseWithCovarianceStamped publisher
* Add PoseWithCovarianceStamped publisher to class 'Autoware'
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* Contributors: Hiroki OTA, Masaya Kataoka, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------

0.4.1 (2021-07-30)
------------------
* Merge pull request `#409 <https://github.com/tier4/scenario_simulator_v2/issues/409>`_ from tier4/feature/autoware/pose-with-covariance
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Update class Autoware to publish LocalizationPose for each frame
* Change topic name of PoseWithCovarianceStamped publisher
* Add PoseWithCovarianceStamped publisher to class 'Autoware'
* Contributors: Masaya Kataoka, yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#407 <https://github.com/tier4/scenario_simulator_v2/issues/407>`_ from tier4/feature/galactic_support
* Cleanup
* Add free function 'doller' emulates shell's '$(...)' expression
* apply reformat
* add ifdef flags for rosdistro
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Move flag 'autoware_initialized' into class 'Autoware'
* Update EgoEntity to occupy one Autoware each
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/test-scenario
* Merge remote-tracking branch 'origin/master' into feature/interpreter/assign-route-action-with-world-position
* Merge pull request `#328 <https://github.com/tier4/scenario_simulator_v2/issues/328>`_ from RobotecAI/pjaroszek/map_and_planning
* killing Autoware: branching for AAP (no changes) and AA (sudokill etc)
* fix formatting
* ArchitectureProposal as default Autoware instead of Auto
* Code review fixes in autoware destructor
* refactor sudokill function
* clang formatting
* adapt formatting
* cleanup comments
* Merge branch 'master' into traffic_signal_actions
* build with AUTOWARE_AUTO flag defined instead of AUTOWARE_ARCHITECTURE_PROPOSAL
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/get_waypoints_error_message
* Merge pull request `#378 <https://github.com/tier4/scenario_simulator_v2/issues/378>`_ from tier4/feature/ego-entity/acuquire-position-action
* Add member function 'description' to syntax UserDefinedValueCondition
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, wjaworski, yamacir-kit

0.2.0 (2021-06-24)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Contributors: kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#335 <https://github.com/tier4/scenario_simulator_v2/issues/335>`_ from tier4/fix-for-galactic
* Fix galactic build errors
* Merge pull request `#312 <https://github.com/tier4/scenario_simulator_v2/issues/312>`_ from tier4/fix/interpreter/acquire-position-action
* Fix Autoware::engage to be asynchronous
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/use_common_exception
* Merge pull request `#307 <https://github.com/tier4/scenario_simulator_v2/issues/307>`_ from tier4/feature/rosbag-record
* Lipsticks
* Update interpreter to start 'ros2 bag record' on configure phase
* Merge pull request `#305 <https://github.com/tier4/scenario_simulator_v2/issues/305>`_ from tier4/refactor/scenario-test-runner
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronize_clock
* Add interactive messages
* Merge pull request `#303 <https://github.com/tier4/scenario_simulator_v2/issues/303>`_ from tier4/feature/common-exception-package
* Remove deprecated error type 'concealer::AutowareError'
* Update concealer to use common::AutowareError
* Merge pull request `#302 <https://github.com/tier4/scenario_simulator_v2/issues/302>`_ from tier4/feature/error-handling-2
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge pull request `#297 <https://github.com/tier4/scenario_simulator_v2/issues/297>`_ from tier4/feature/error-handling
* Update Autoware::~Autoware to kill launch process if no respond
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Replace some of std::cout with RCLCPP_INFO_STREAM
* Contributors: Kenji Miyake, Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#294 <https://github.com/tier4/scenario_simulator_v2/issues/294>`_ from tier4/feature/support-autoware.iv-0.11.2
  Feature/support autoware.iv 0.11.2
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Update to call setLaneChangeApproval only once
* Update Autoware's transition max duration to 20 sec from 10 sec
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#293 <https://github.com/tier4/scenario_simulator_v2/issues/293>`_ from tier4/hotfix/disable-emergency-monitoring
  Disable emergency monitoring
* Disable emergency monitoring
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* Lipsticks
* Merge pull request `#287 <https://github.com/tier4/scenario_simulator_v2/issues/287>`_ from tier4/feature/remove-dummy-perception-publisher
  Feature/remove dummy perception publisher
* Update struct Autoware to catch exception from node spinner
* Rename macro identifier '_IV' to '_ARCHITECTURE_PROPOSAL'
* Rename package 'awapi_accessor' to 'concealer'
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit
