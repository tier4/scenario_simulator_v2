^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scenario_test_runner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2022-01-11)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge pull request `#641 <https://github.com/tier4/scenario_simulator_v2/issues/641>`_ from tier4/feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* fix all-in-one scenario to verify the evaluation of the expression
* Merge branch 'feature/avoid_overwrite_acceleration' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Comment-out changes
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Fix `TransitionAssertion` to stop if class `Autoware` down
* Fix `waitForAutowareStateToBe*` to call thunk at least one time.
* Update `initialize_duration` to `50` from `30` (for Autoware.Universe)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Remove `autoware_auto_msgs` from dependency
* Set default `architecture_type` to `tier4/proposal`
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* some changes to run psim with autoware_universe
* Contributors: MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_newton_method_from_get_s_value
* Merge pull request `#605 <https://github.com/tier4/scenario_simulator_v2/issues/605>`_ from tier4/refactor/interpreter/reference
* Update `lookupQualifiedElement` argument iterators to not to include variable name
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Update test scenario `prefixed-name-reference.yaml`
* Add new test scenario `prefixed-name-reference.yaml`
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/get_s_value
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/function-name
* Merge pull request `#579 <https://github.com/tier4/scenario_simulator_v2/issues/579>`_ from tier4/feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* fix setup.py and catalog path in scenario files
* fix scenario_test_runner/setup.py for catalog
* Merge branch 'master' into feature/interpreter/catalog
* add catalog test
* add CatalogReference
* Merge branch 'master' into feature/interpreter/catalog
* catalog parameter
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* convert scenario file from yaml to xosc
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.5.6 (2021-10-28)
------------------
* Merge pull request `#592 <https://github.com/tier4/scenario_simulator_v2/issues/592>`_ from tier4/fix/version
* update version
* Merge pull request `#582 <https://github.com/tier4/scenario_simulator_v2/issues/582>`_ from alexandrx/fix/rviz2-config
* Fixed error in RVIZ2 config path in launch file
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge pull request `#567 <https://github.com/tier4/scenario_simulator_v2/issues/567>`_ from tier4/feature/interpreter/user-defined-value-condition
* Move some messages into new package `openscenario_msgs`
* Fix `UserDefinedValueCondition` to support to receive multiple message
* Update `UserDefinedValueCondition` to return false if no message received
* Update `UserDefinedValueCondition` to receive message
* Add new message type `ParameterDeclaration` and `ParameterType`
* Update `UserDefinedValueCondition` to reverive name of path-like pattern
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Contributors: Alexander Carballo, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------

0.5.4 (2021-10-13)
------------------
* Merge pull request `#557 <https://github.com/tier4/scenario_simulator_v2/issues/557>`_ from tier4/revert/pr_544
* Revert "Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status"
* Merge pull request `#554 <https://github.com/tier4/scenario_simulator_v2/issues/554>`_ from tier4/feature/autoware/upper-bound-velocity
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Fix Autoware's default upper bound speed to double max from zero
* Update syntax `ObjectController` to support `assign` member function
* Update syntax `AssignControllerAction` to be optional
* Update syntax `OverrideControllerValueAction` to be optional
* Fix `setVehicleVelocity` to work in `Autoware::update`
* Update Property/Properties operator []
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------

0.5.2 (2021-10-06)
------------------
* Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status
* move rviz file and configure depends
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#527 <https://github.com/tier4/scenario_simulator_v2/issues/527>`_ from tier4/fix/clean_directory_behavior
* Merge pull request `#528 <https://github.com/tier4/scenario_simulator_v2/issues/528>`_ from RobotecAI/add_demo_scenario_that_works_with_autoware_auto
* Merge branch 'master' into fix/clean_directory_behavior
* add dedicated scenario for AutowareAuto
* Merge pull request `#525 <https://github.com/tier4/scenario_simulator_v2/issues/525>`_ from RobotecAI/rename_AA_launch_package
* Merge branch 'master' into rename_AA_launch_package
* Merge pull request `#491 <https://github.com/tier4/scenario_simulator_v2/issues/491>`_ from tier4/feature/interpreter/fault-injection
* Update `FaultInjectionAction` topic name to `/simulation/events`
* change log directory cleanup behavior
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge pull request `#524 <https://github.com/tier4/scenario_simulator_v2/issues/524>`_ from tier4/fix/get-jerk-limit-from-object-controller
* rename scenario_test_runner_launch to scenario_simulator_launch
* fix snake_case to lowerCamelCase
* get jerk limits from ObjectController's property
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/standstill_metric
* Merge remote-tracking branch 'origin/master' into feature/interpreter/distance-condition
* Merge pull request `#506 <https://github.com/tier4/scenario_simulator_v2/issues/506>`_ from tier4/feature/interpreter/add-entity-action
* Update `AddEntityAction` to treat various Position types
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Add new test scenario `CustomCommandAction.FaultInjectionAction.yaml`
* Cleanup test scenario `autoware-simple.yaml`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Support new UserDefinedValueCondition `<ENTITY-NAME>.currentState`
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* change to count-up
* fix typo in rviz
* fix typo
* change to count_up
* fix typo of reaches
* fix some typo
* update HOGE/FUGA
* use foo/bar/baz
* Revert "Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel"
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* Merge pull request `#490 <https://github.com/tier4/scenario_simulator_v2/issues/490>`_ from tier4/fix/scenario-object-scope
* add a new scenario to check duplicated parameter
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Merge branch 'master' into add-goalpose
* Support new option `record:=<boolean>`
* Support new option `initialize_duration`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Add new launch-argument `launch-autoware:=<boolean>`
* Lipsticks
* Rename launch-argument `with-rviz` to `launch_rviz`
* Rename launch-argument `output-directory` to `output_directory`
* Rename launch-arugment `global-timeout` to `global_timeout`
* Rename launch-argument `global-real-time-factor` to `global_real_time_factor`
* Rename launch-arugment `global-frame-rate` to `global_frame_rate`
* Rename option `autoware-launch-package` to `autoware_launch_package`
* Rename option `autoware-launch-file` to `autoware_launch_file`
* Rename option `architecture-type` to `architecture_type`
* Update scenario_test_runner.launch.py options to align vertically
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* update rviz
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, kyabe2718, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Merge pull request `#437 <https://github.com/tier4/scenario_simulator_v2/issues/437>`_ from RobotecAI/issue/AJD-237-remove_autoware_compilation_flag
* review changes
* cleanup
* autoware auto launch dependency in scenario test runner
* make Autoware switch based on autoware_type parameter
* introduce ROS param for autoware_type
* switch to AutowareAuto
* code review fixes
* Merge pull request `#444 <https://github.com/tier4/scenario_simulator_v2/issues/444>`_ from tier4/feature/interpreter/cleanup-error-messages
* Fix SyntaxError `Init.Actions should end immediately` of test scenario `Property.isBlind`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Add member function 'Interpreter::publishCurrentContext'
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Lipsticks
* Merge branch 'master' into AJD-238_scenario_validation
* removed validation from scenario_test_runner
* removed unused import
* added test scenario with different routing goal and end condition
* ReachPositionConditionValidator added
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/suppress_warnings
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/follow_front_entity_behavior
* Merge pull request `#430 <https://github.com/tier4/scenario_simulator_v2/issues/430>`_ from tier4/feature/interpreter/error-message
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#420 <https://github.com/tier4/scenario_simulator_v2/issues/420>`_ from tier4/namespace
* Update script 'result_checker'
* fix bugs of name resolution with anonymous scope and change all-in-one.yaml to require name resolution
* Merge remote-tracking branch 'origin/master' into namespace
* Resolve warnings from setuptools
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* trivial fix
* fix traffic signals
* remove test dir
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------

0.4.1 (2021-07-30)
------------------
* Merge pull request `#418 <https://github.com/tier4/scenario_simulator_v2/issues/418>`_ from tier4/feature/add_cpp_scenario_node
* apply reformat and with_rviz argument
* Merge pull request `#417 <https://github.com/tier4/scenario_simulator_v2/issues/417>`_ from tier4/feature/add_mock_scenarios
* remove test dir
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Contributors: Masaya Kataoka, yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#407 <https://github.com/tier4/scenario_simulator_v2/issues/407>`_ from tier4/feature/galactic_support
* Lipsticks
* Add free function 'doller' emulates shell's '$(...)' expression
* fix rclpy.init API usage
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Lipsticks
* Update EgoEntity to occupy one Autoware each
* Remove debug codes from EgoEntity
* Add member function 'get*MapFile' to struct Configuration
* Update EgoEntity's constructor to receive Configuration
* Lipsticks
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* Merge pull request `#401 <https://github.com/tier4/scenario_simulator_v2/issues/401>`_ from tier4/fix/typo
* fix typo described in https://github.com/tier4/scenario_simulator_v2/issues/398
* Merge remote-tracking branch 'origin/master' into fix/interpreter/acquire-position-action
* Merge pull request `#390 <https://github.com/tier4/scenario_simulator_v2/issues/390>`_ from tier4/feature/interpreter/traffic-signal-controller-condition
* Update test scenario 'all-in-one'
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge pull request `#386 <https://github.com/tier4/scenario_simulator_v2/issues/386>`_ from tier4/feature/interpreter/misc-object
* Fix rviz config paths
* Fix Storyboard to check entities are ready if only before start simulation
* Rename rviz configuration file
* Simplify scenario 'all-in-one'
* Update test scenario 'all-in-one' to use MiscObject
* Add utility function 'overload'
* Merge pull request `#383 <https://github.com/tier4/scenario_simulator_v2/issues/383>`_ from tier4/feature/interpreter/test-scenario
* Support TrafficSignalCondition
* Lipsticks
* Add TrafficSignalControllers to scenario 'all-in-one'
* Merge pull request `#385 <https://github.com/tier4/scenario_simulator_v2/issues/385>`_ from tier4/fix/remove-unresolvable-dependency
* Remove dependency for only Autoware.Auto (to fix rosdep error)
* Merge remote-tracking branch 'origin/master' into feature/interpreter/test-scenario
* Merge remote-tracking branch 'origin/master' into feature/interpreter/assign-route-action-with-world-position
* Merge pull request `#328 <https://github.com/tier4/scenario_simulator_v2/issues/328>`_ from RobotecAI/pjaroszek/map_and_planning
* Merge branch 'master' into pjaroszek/map_and_planning
* Add scenario 'all-in-one' to example workflow
* Update scenario 'all-in-one' to be successfull
* ArchitectureProposal as default Autoware instead of Auto
* Merge pull request `#377 <https://github.com/tier4/scenario_simulator_v2/issues/377>`_ from tier4/traffic_signal_actions
* fix bug
* trivial fix
* add support for TrafficSignalController.referece
* rebase adjustments
* Merge branch 'master' into traffic_signal_actions
* build with AUTOWARE_AUTO flag defined instead of AUTOWARE_ARCHITECTURE_PROPOSAL
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/get_waypoints_error_message
* Merge pull request `#378 <https://github.com/tier4/scenario_simulator_v2/issues/378>`_ from tier4/feature/ego-entity/acuquire-position-action
* Update EgoEntity to be able to request AcquirePositionAction multiple times
* Add struct 'UserDefinedValueCondition'
* Merge branch 'master' into traffic_signal_actions
* add support for TrafficSingnalAction
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.2.0 (2021-06-24)
------------------
* Merge pull request `#370 <https://github.com/tier4/scenario_simulator_v2/issues/370>`_ from tier4/feature/interpreter/context-2
* Lipsticks
* Add member function 'description' to syntax StoryboardElementStateCondition
* Add member function 'description' to syntax ParameterCondition
* Add member function 'distance' to syntax RelativeDistanceCondition
* Add member function 'description' to syntax DistanceCondition
* Add member function 'description' to syntax ReachPositionCondition
* Add member function 'description' to syntax SpeedCondition
* Add member function 'description' to syntax StandStillCondition
* Add member function 'description' to syntax AccelerationCondition
* Add member function 'description' to syntax TimeHeadwayCondition
* Add member function 'description' to syntax CollisionCondition
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka, yamacir-kit

0.1.1 (2021-06-21)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge pull request `#320 <https://github.com/tier4/scenario_simulator_v2/issues/320>`_ from tier4/relative_target_speed
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge branch 'feature/interpreter/context' of github.com:tier4/scenario_simulator_v2 into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Actions in Init must be completed immediately
* Merge branch 'master' into relative_target_speed
* Merge branch 'master' into relative_target_speed
* add relative_target_speed.yaml
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/get_vehicle_cmd
* Merge pull request `#354 <https://github.com/tier4/scenario_simulator_v2/issues/354>`_ from tier4/fix/typos-misc
* Fix typos in docs / mock / simulation/ test_runner
* Merge pull request `#346 <https://github.com/tier4/scenario_simulator_v2/issues/346>`_ from tier4/feature/replay.launch
* add launch file
* Merge pull request `#341 <https://github.com/tier4/scenario_simulator_v2/issues/341>`_ from tier4/fix/traffic-simulator/vehicle-description
* Cleanup
* Update launch file to receive LaunchContext
* Merge pull request `#338 <https://github.com/tier4/scenario_simulator_v2/issues/338>`_ from tier4/feature/interpreter/vehicle-description
* Fixed not to load description when no argument vehicle is given
* Update EgoEntity to use precise simulation model parameters
* Update scenario_test_runner.launch.py to load Autoware parameter
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* Merge branch 'master' into fix/typos-in-docs-and-comments
* Fix typos and grammars in docs and comments
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#331 <https://github.com/tier4/scenario_simulator_v2/issues/331>`_ from tier4/feature/interpreter/traffic-signals
* Cleanup struct 'OpenScenario'
* Move struct ScenarioDefinition into new header
* Rename scenario 'Autoware.TrafficSignals' to 'TrafficSignals'
* Update test scenario
* Merge pull request `#309 <https://github.com/tier4/scenario_simulator_v2/issues/309>`_ from tier4/fix/interpreter/deactivation
* Add new test scenario 'empty'
* Merge pull request `#308 <https://github.com/tier4/scenario_simulator_v2/issues/308>`_ from tier4/feature/check_rosbag_output
* fix typo
* apply reformat
* enable check log output
* Merge pull request `#305 <https://github.com/tier4/scenario_simulator_v2/issues/305>`_ from tier4/refactor/scenario-test-runner
* Add interactive messages
* Remove deprecated launch files
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge remote-tracking branch 'origin/master' into feature/error-handling
* Contributors: Kazuki Miyahara, Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#295 <https://github.com/tier4/scenario_simulator_v2/issues/295>`_ from tier4/fix/python_format
  reformat by black
* reformat by black
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* fix licence text
* remove flake8 check
* apply format
* modify import order
* use single quate
* add new line for the block
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* Update scenario_test_runner.launch.py to receive sensor and vehicle model
* Update RViz configuration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/update_contact_information
* Merge pull request `#284 <https://github.com/tier4/scenario_simulator_v2/issues/284>`_ from tier4/remove-use-sim-time
  Remove use sim time
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-0.11.1
* Merge pull request `#281 <https://github.com/tier4/scenario_simulator_v2/issues/281>`_ from tier4/feature/asynchronous-autoware-initialization
  Feature/asynchronous autoware initialization
* Update Storyboard to call engage if Autoware is ready (= WaitingForEngage)
* Remove use sim time
* Lipsticks
* Merge pull request `#283 <https://github.com/tier4/scenario_simulator_v2/issues/283>`_ from tier4/feature/add_with_rviz_option
  add option file
* enable pass colcon test
* add option file
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/update_contact_information
* Merge pull request `#276 <https://github.com/tier4/scenario_simulator_v2/issues/276>`_ from tier4/feature/autoware-high-level-api
  Feature/autoware high level api
* Move Autoware process control into class 'Autoware'
* Merge pull request `#274 <https://github.com/tier4/scenario_simulator_v2/issues/274>`_ from tier4/refactor/cleanup-ego-entity
  Refactor/cleanup ego entity
* Move EgoEntity::EgoEntity into ego_entity.cpp
* Merge github.com:tier4/scenario_simulator.auto into feature/change_base_image
* Merge branch 'master' into feature/support-autoware.iv-0.11.1
* Merge pull request `#266 <https://github.com/tier4/scenario_simulator_v2/issues/266>`_ from tier4/feature/interpreter/traffic-signal-controller-3
  Feature/interpreter/traffic signal controller 3
* Update EgoEntity to launch Autoware via autoware_launch
* Unlock InfrastructureAction
* Update readElement to return std::list instead of std::vector
* Lipsticks
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpolate_two_center_points
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#263 <https://github.com/tier4/scenario_simulator_v2/issues/263>`_ from tier4/feature/traffic-signal-sensor
  Feature/traffic signal sensor
* Update traffic signals topic name to use AWAPI
* Merge https://github.com/tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge pull request `#262 <https://github.com/tier4/scenario_simulator_v2/issues/262>`_ from tier4/feature/interpreter/traffic-signal-controller-2
  Feature/interpreter/traffic signal controller 2
* Update TrafficSignalState to invoke API 'setTrafficLightColor'
* Merge branch 'master' into feature/interpreter/traffic-signal-controller
* Merge pull request `#258 <https://github.com/tier4/scenario_simulator_v2/issues/258>`_ from tier4/fix/misc-problems
  Fix/misc problems
* Lipsticks
* Merge remote-tracking branch 'origin/fix/misc-problems' into feature/interpreter/traffic-signal-controller
* Remove some scenarios depend deprecated interpreter's behavior
* Move test scenarios into one level higher directory
* Update iota function to use same algorithm with CI/CD
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* fix launch file
* rename package
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* rename simulation_api package
* Merge branch 'master' into feature/publish_proto_doc
* Merge pull request `#251 <https://github.com/tier4/scenario_simulator_v2/issues/251>`_ from tier4/feature/support-autoware.iv/rc-0.11.0
  Feature/support autoware.iv/rc 0.11.0
* Update launch files to match that of autoware_launch
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#247 <https://github.com/tier4/scenario_simulator_v2/issues/247>`_ from tier4/fix-for-rolling
  Replace doc by description
* Replace doc by description
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* Merge branch 'master' into feature/assign_route_action
* Merge pull request `#242 <https://github.com/tier4/scenario_simulator_v2/issues/242>`_ from tier4/feature/interpreter/remove-short-circuit-evaluation
  Feature/interpreter/remove short circuit evaluation
* Update Trigger/ConditionGroup to re-evaluate children for each evaluation
* Add helper function 'apply' to dispatch EntityObject dynamically
* Merge branch 'master' into fix/reindex-rtree
* Merge pull request `#234 <https://github.com/tier4/scenario_simulator_v2/issues/234>`_ from tier4/feature/interpreter/teleport-action
  Feature/interpreter/teleport action
* Replace constructLaneletPose with LanePosition type's cast operator
* Merge pull request `#232 <https://github.com/tier4/scenario_simulator_v2/issues/232>`_ from tier4/misc
  Misc
* Update rviz configuration
* Merge pull request `#229 <https://github.com/tier4/scenario_simulator_v2/issues/229>`_ from tier4/feature/test-runner/autoware.launch.xml
  Feature/test runner/autoware.launch.xml
* Cleanup openscenario_interpreter.cpp
* Rename some parameters
* Update EgoEntity to receive Autoware launch file via parameter
* Move help messages into launch.py
* Update functor 'launch_autoware'
* Merge branch 'master' into doc/zeromq
* Cleanup EgoEntity's constructor
* Merge pull request `#227 <https://github.com/tier4/scenario_simulator_v2/issues/227>`_ from tier4/feature/interpreter/object-controller
  Feature/interpreter/object controller
* Update test scenarios
* Update Controller.Properties to support property 'isEgo'
* Merge pull request `#225 <https://github.com/tier4/scenario_simulator_v2/issues/225>`_ from tier4/feature/support-autoware.iv-9
  Feature/support autoware.iv 9
* Fix raycasting
* Add new test scenario 'Autoware.Overtake'
* Add rviz config file
* Update EgoEntity to redirect Autoware's output to file
* Merge https://github.com/tier4/scenario_simulator.auto into doc/zeromq
* Update ros2 bag record output to be log level
* Merge pull request `#223 <https://github.com/tier4/scenario_simulator_v2/issues/223>`_ from tier4/feature/add_rosbag_record
  feat: rosbag executable commnad
* feat: rosbag executable commnad
* Merge pull request `#222 <https://github.com/tier4/scenario_simulator_v2/issues/222>`_ from tier4/feature/interpreter/sticky
  Feature/interpreter/sticky
* Add condition edge 'sticky'
* Cleanup autoware.launch.xml
* Merge pull request `#220 <https://github.com/tier4/scenario_simulator_v2/issues/220>`_ from tier4/feature/support-autoware.iv-8
  Feature/support autoware.iv 8
* Remove system_monitor from launch
* Merge branch 'master' into feature/remove_xmlrpc
* Merge branch 'master' into feature/zeromq_integration
* Merge pull request `#218 <https://github.com/tier4/scenario_simulator_v2/issues/218>`_ from tier4/feature/support-autoware.iv-7
  Feature/support autoware.iv 7
* Add topic namespace 'awapi'
* Fix autoware.launch to match Proposal.iv upstream
* Merge pull request `#215 <https://github.com/tier4/scenario_simulator_v2/issues/215>`_ from tier4/feature/support-autoware.iv-6
  Update EgoEntity to pass map_path to launch file
* Update EgoEntity to pass map_path to launch file
* Merge pull request `#214 <https://github.com/tier4/scenario_simulator_v2/issues/214>`_ from tier4/feature/support-autoware.iv-5
  Feature/support autoware.iv 5
* Add some type aliases
* Cleanup EntityManager's constructor
* Update variable 'count' of EgoEntity::waitForAutowareToBeReady to be non-static
* Merge pull request `#208 <https://github.com/tier4/scenario_simulator_v2/issues/208>`_ from tier4/feature/support-autoware.iv-4
  Feature/support autoware.iv 4
* Update AWAPI Accessor to consider multi-time instantiation
* Fix map-path
* Update EgoEntity to kill launched Autoware
* Update scenario_test_runner.launch.py to launch autoware
* Update EgoEntity to terminate Accessor
* Update EntityManager to prevent to spawn an entity more than once
* Update EgoEntity to launch autoware
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into fix/xmlrpc_connection_lost
* Merge pull request `#206 <https://github.com/tier4/scenario_simulator_v2/issues/206>`_ from tier4/fix/node-duplication
  Fix/node duplication
* Update EgoEntity to make Accessor node with 'use_global_arguments(false)'
* Update nodes namespace
* Fix LifecycleController's node name
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/visualize_traffic_light_stae
* Merge pull request `#204 <https://github.com/tier4/scenario_simulator_v2/issues/204>`_ from tier4/feature/support-autoware.iv-3
  Feature/support autoware.iv 3
* Fix message type of '/vehicle/status/steering'
* Merge branch 'master' into feature/get_waypoint_from_autoware
* Merge branch 'master' into feature/get_waypoint_from_autoware
* Merge pull request `#197 <https://github.com/tier4/scenario_simulator_v2/issues/197>`_ from tier4/feature/support-autoware.iv-2
  Feature/support autoware.iv 2
* Add missing relay
* Update ego_entity to initialize Autoware correctly
* Lipsticks
* Fix topic type for AutowareEngage
* Remove unused publisher/subscription from API class
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/lidar_simulation
* Rename parameter 'log_path' to 'output_directory'
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#199 <https://github.com/tier4/scenario_simulator_v2/issues/199>`_ from tier4/feature/controller
  Feature/controller
* Fix AssignControllerAction's bug
* Add test scenaro for AssignControllerAction
* Fix bag
* Support ControllerAction
* Add test scenario 'blind.yaml'
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Update ego entity's member function 'onUpdate'
* Add simulation specific topics to Accessor
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/use_protobuf_in_spawn
* Merge pull request `#179 <https://github.com/tier4/scenario_simulator_v2/issues/179>`_ from tier4/feature/support-autoware.iv
  Feature/support autoware.iv
* Add a test scenario uses property 'isEgo'
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/protobuf_xmlrpc
* Merge branch 'master' into feature/support-autoware.iv
* Merge pull request `#191 <https://github.com/tier4/scenario_simulator_v2/issues/191>`_ from tier4/feature/interpreter/property
  Feature/interpreter/property
* Remove property 'isEgo' from some test scenarios
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv
* Merge pull request `#174 <https://github.com/tier4/scenario_simulator_v2/issues/174>`_ from tier4/feature/destroy-entity-action
  Feature/destroy entity action
* Merge branch 'fix/metrics' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_crosswalk
* Update test scenario 'simple.xosc'
* Merge pull request `#167 <https://github.com/tier4/scenario_simulator_v2/issues/167>`_ from tier4/feature/custom-command-action
  Feature/custom command action
* Lipsticks
* Update CustomCommandAction to accept C-style function syntax
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/obstacle_visualization
* Merge pull request `#157 <https://github.com/tier4/scenario_simulator_v2/issues/157>`_ from tier4/feature/scenario-test-runner-options
  Feature/scenario test runner options
* Cleanup
* Support single scenario execution
* Update function 'run_scenarios'
* Rename option from 'log_directory' to 'output_directory'
* Update to use 'log_directory' as convert_scenarios output_directory
* Move function 'convert_scenarios' into scenario_test_runner.py
* Rewrite scenario conversion
* Update function 'convert_scenarios' to receive list of 'Scenario'
* Add class 'Scenario' and 'Expect'
* Update help messages
* Change type of option '--workflow' to Path
* Lipsticks
* Remove deprecated option 'no_validation'
* Merge branch 'master' into feature/scenario-test-runner-options
* Merge pull request `#149 <https://github.com/tier4/scenario_simulator_v2/issues/149>`_ from tier4/feature/foxy
  Feature/foxy
* Fix test scenario 'simple.xosc'
* Add option '--scenario' to launch.py
* Add option '--global-timeout' to launch.py
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge pull request `#150 <https://github.com/tier4/scenario_simulator_v2/issues/150>`_ from tier4/feature/entity_waypoint
  Feature/entity waypoint
* remove failure scenario
* Update option 'global-timeout' to disable if None specified
* Lipsticks
* Rename option 'timeout' to 'global-timeout'
* Add optional argument verbose (= True) to openscenario_utility
* Rename 'step_time_ms' to 'frame-rate'
* Fix external/quaternion_operation's commit hash
* Add parameter 'real-time-factor' to openscenario_interpreter
* Rename option to '--global-real-time-factor' from 'real-time-factor'
* Add launch argument '--real-time-factor'
* Cleanup
* Fix launch file
* configure origin
* change default parameters
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/entity_waypoint
* Merge pull request `#148 <https://github.com/tier4/scenario_simulator_v2/issues/148>`_ from tier4/refactor/scenario-test-runner-2
  Refactor/scenario test runner 2
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Add missing dependency
* Add package.xml to package 'openscenario_utility'
* Remove package 'scenario_test_utility'
* Move workflow_validator into class Workflow
* Lipsticks
* Fix docstrings
* Merge branch 'master' into refactor/scenario-test-runner-2
* Merge pull request `#147 <https://github.com/tier4/scenario_simulator_v2/issues/147>`_ from tier4/feature/remove_entity_status
  Feature/remove entity status
* Add constructor to class 'Workflow'
* Move workflow-file path resolution to outside of class 'Workflow'
* Update permissions
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* Rename class 'DatabaseHandler' to 'Workflow'
* fix problems in lane change action
* Merge pull request `#146 <https://github.com/tier4/scenario_simulator_v2/issues/146>`_ from tier4/refactor/scenario-test-runner
  Refactor/scenario test runner
* Remove class 'Manager' from dependency of module 'DatabaseHandler'
* Replace Manager.read_data
* Cleanup
* Convert log_path to type Path from str
* Move log directory resolution to TestRunner from DatabaseHandler
* Replace Logger with ROS2 logger
* Remove module 'regex'
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* Remove return value 'launcher_path' from DatabaseHandler
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* Merge pull request `#144 <https://github.com/tier4/scenario_simulator_v2/issues/144>`_ from tier4/feature/ros-independent-converter
  Feature/ros independent converter
* Lipsticks
* Replace validator to use refactored version
* Remove 'convert.py' of package 'scenario_test_utility'
* Replace scenario converter to ROS2-independent version
* Merge branch 'master' into feature/collision_to_hermite_curve
* Merge pull request `#128 <https://github.com/tier4/scenario_simulator_v2/issues/128>`_ from tier4/feature/ordered-xosc
  Feature/ordered xosc
* Lipsticks
* Replace scenario conversion to use new converter 'convert.py'
* Add class 'MacroExpander'
* Merge branch 'master' into feature/ordered-xosc
* Merge pull request `#136 <https://github.com/tier4/scenario_simulator_v2/issues/136>`_ from tier4/feature/remove_scenario_simulator_msgs
  Feature/remove scenario simulator msgs
* remove unused packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_spline_interpolation
* Merge pull request `#133 <https://github.com/tier4/scenario_simulator_v2/issues/133>`_ from tier4/feature/relative-world-position
  Feature/relative world position
* move directory
* Update Orientation type to support implict cast to Vector3
* Add utility function 'fold\_(left|right)' and 'cat'
* Merge branch 'master' into feature/spawn_relative_entity_position
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/catmull-rom
* Merge pull request `#118 <https://github.com/tier4/scenario_simulator_v2/issues/118>`_ from tier4/document/openscenario_interpreter
  Document/openscenario interpreter
* Merge remote-tracking branch 'origin/master' into document/openscenario_interpreter
* Merge branch 'master' into rosdep/python3-bs4
* Merge pull request `#120 <https://github.com/tier4/scenario_simulator_v2/issues/120>`_ from tier4/hotfix/default-behavior
  Change default behavior of option --no-validation
* Revert a change
* Invert default value of option 'use_validation'
* Change default to use no validation
* Merge pull request `#119 <https://github.com/tier4/scenario_simulator_v2/issues/119>`_ from tier4/feature/add_validation_option
  add validation option
* Change default to use no validation
* Fix bug
* Update substitution syntax description
* add validation option
* Merge pull request `#114 <https://github.com/tier4/scenario_simulator_v2/issues/114>`_ from tier4/doc/test_runner
  Doc/test runner
* Merge branch 'master' into doc/test_runner
* add doc for test runner
* add doc for lifecycle
* refactor docstring
* refactor docstring
* refactor docstring
* add ament docstring
* Merge pull request `#117 <https://github.com/tier4/scenario_simulator_v2/issues/117>`_ from tier4/document/openscenario_interpreter
  Document/openscenario interpreter
* refactor codes
* add entry point and comments
* Fix company name
* Merge branch 'master' into feature/awapi_adapter/add_info
* Merge branch 'master' into feature/awapi_adapter/vehicle_info
* Merge pull request `#94 <https://github.com/tier4/scenario_simulator_v2/issues/94>`_ from tier4/fix/contact_infomation
  Fix/contact infomation
* Merge remote-tracking branch 'origin/master' into feature/awapi_adapter/add_info
* Merge branch 'master' into feature/awapi_adapter/vehicle_info
* Merge pull request `#93 <https://github.com/tier4/scenario_simulator_v2/issues/93>`_ from tier4/fix/copyright
  update copyright
* fix contact infomation of taiki tanaka
* update copyright
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into documentation/simulation_api
* Merge pull request `#78 <https://github.com/tier4/scenario_simulator_v2/issues/78>`_ from tier4/fix/collision
  Fix/collision
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into fix/collision
* Merge pull request `#74 <https://github.com/tier4/scenario_simulator_v2/issues/74>`_ from tier4/feature/procedures
  Feature/procedures
* Add new test scenario 'distance-condition.yaml'
* use disjoint algorithun
* update collision check algorithum
* Merge branch 'feature/procedures' of https://github.com/tier4/scenario_simulator.auto into fix/collision
* update workflow
* Add new test scenario 'stand-still.yaml'
* add collision.yaml
* Add new test scenario 'collsion.yaml'
* Merge pull request `#72 <https://github.com/tier4/scenario_simulator_v2/issues/72>`_ from tier4/fix/documentation
  update documentation
* Merge branch 'master' into fix/documentation
* Merge pull request `#71 <https://github.com/tier4/scenario_simulator_v2/issues/71>`_ from tier4/feature/parameter
  Feature/parameter
* update documentation
* Rename word from 'open_scenario' to 'openscenario'
* Merge remote-tracking branch 'origin/master' into feature/parameter
* Lipsticks
* Remove parameter map_path
* Merge https://github.com/tier4/scenario_simulator.auto into feature/export_docker
* Fix scenario_test_runner's parameter update
* Merge branch 'master' into feature/export_docker
* Merge pull request `#67 <https://github.com/tier4/scenario_simulator_v2/issues/67>`_ from tier4/feature/test_runner/add_implementation_details
  add latest readme
* Merge branch 'master' into feature/test_runner/add_implementation_details
* Merge pull request `#66 <https://github.com/tier4/scenario_simulator_v2/issues/66>`_ from tier4/refactor/converter
  Refactor/converter
* add editor readme
* Update sample scenario 'minimal'
* Update member function 'guard'
* remove debug info
* Cleanup
* Lipsticks
* add latest readme
* Merge branch 'master' into refactor/converter
* Merge https://github.com/tier4/scenario_simulator.auto into feature/yield
* Merge branch 'master' into feature/yield
* Merge pull request `#58 <https://github.com/tier4/scenario_simulator_v2/issues/58>`_ from tier4/refactor/interpreter/error-handling
  Refactor/interpreter/error handling
* Update converter to ignore empty list
* Simplify ScenarioConverter
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/error-handling
* Merge pull request `#61 <https://github.com/tier4/scenario_simulator_v2/issues/61>`_ from tier4/feature/stop_at_stopline
  Feature/stop at stopline
* Merge pull request `#59 <https://github.com/tier4/scenario_simulator_v2/issues/59>`_ from tier4/feature/enhance_visualization
  Feature/enhance visualization
* enable load map in mock
* enable visualize marker
* enable configure visualization status
* apply reformat and modify launch files
* add visualization node
* Update Storyboard initialization
* Update error message
* Apply guard to initialization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/refactor_behavior_tree_architecture
* Merge remote-tracking branch 'origin/master' into feature/interpreter/scope
* Merge pull request `#55 <https://github.com/tier4/scenario_simulator_v2/issues/55>`_ from tier4/feature/set_loop_rate
  Feature/set loop rate
* enable pass ament_copyright
* enable pass ament_flake8
* enable pass step time via workflow
* change expect field into optional
* enabe set step_time via rosparam
* Merge pull request `#48 <https://github.com/tier4/scenario_simulator_v2/issues/48>`_ from tier4/combine/interpreter_and_simulator
  Connect/interpreter to simulator
* Add debug code (for CI)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/dockerhub_integration
* Update to undeclare_parameter after get_parameter
* Rename parameter 'scenario' to 'osc_path'
* Update setup.py to install test scenarios into nested directory
* Merge remote-tracking branch 'origin/master' into combine/interpreter_and_simulator
* Merge pull request `#53 <https://github.com/tier4/scenario_simulator_v2/issues/53>`_ from tier4/feature/scenario_testing
  Feature/scenario testing
* Fix typo
* Merge pull request `#52 <https://github.com/tier4/scenario_simulator_v2/issues/52>`_ from tier4/feature/validate_workflow
  Feature/validate workflow
* Merge branch 'feature/scenario_testing' of https://github.com/tier4/scenario_simulator.auto into feature/dockerhub_integration
* enable write find-pkg-share in workflow file
* updae workdlow
* add result_checker
* fix problems in python
* Merge branch 'master' into feature/validate_workflow
* shutdown scenario test runner when the workflow file is not valid
* add workflow validator
* update workflow_example.yaml
* Update sample scenario 'simple.xosc'
* Fix some missing connections
* Merge remote-tracking branch 'origin/master' into combine/interpreter_and_simulator
* Update scenario_test_runner.launch.py
* Merge pull request `#50 <https://github.com/tier4/scenario_simulator_v2/issues/50>`_ from tier4/feature/launch_argument
  enable pass log directory via launch argument
* enable pass colcon test
* Merge remote-tracking branch 'origin/master' into combine/interpreter_and_simulator
* enable pass log directory via launch argument
* Merge pull request `#49 <https://github.com/tier4/scenario_simulator_v2/issues/49>`_ from tier4/feature/launch_argument
  Feature/launch argument
* fix indent
* enable set package via command line
* Update to instantiate 'simulation_api::API'
* add command line argument to launch file
* enable pass workflow via launch
* fix database handler
* add failure.yml
* Merge pull request `#45 <https://github.com/tier4/scenario_simulator_v2/issues/45>`_ from tier4/refactor/open_scenario_interpreter
  Refactor/open scenario interpreter
* Lipsticks
* Merge remote-tracking branch 'origin/master' into refactor/open_scenario_interpreter
* enable publish junit result
* Rename 'scenario_runner' to 'open_scenario_interpreter'
* Merge pull request `#44 <https://github.com/tier4/scenario_simulator_v2/issues/44>`_ from tier4/feature/scenario_runner/substitution_syntax
  Feature/scenario runner/substitution syntax
* Merge branch 'master' into feature/scenario_runner/substitution_syntax
* update workflow yaml format
* Support builtin substitution syntax 'dirname'
* Support string-interpolation
* Merge branch 'master' into feature/junit_exporter
* Merge pull request `#41 <https://github.com/tier4/scenario_simulator_v2/issues/41>`_ from tier4/feature/scenario_runner/parameter
  Feature/scenario runner/parameter
* Fix stopTransition's bug
* Update to maximumExecutionCount works
* Add class ParameterCondition
* Merge remote-tracking branch 'origin/master' into feature/scenario_runner/parameter
* Merge pull request `#42 <https://github.com/tier4/scenario_simulator_v2/issues/42>`_ from tier4/feature/add_xosc_validation
  Feature/add xosc validation
* Rename SetAction to ParameterSetAction
* enable validate scenarios before running test case
* Update (Set|Modify)Action to receive attribute 'parameterRef'
* add ament_openscenario package
* fix success.yaml
* Merge branch 'master' into feature/scenario_runner/parameter
* Add class 'ModifyAction'
* Merge pull request `#40 <https://github.com/tier4/scenario_simulator_v2/issues/40>`_ from tier4/feature/xosc_validation
  add xsd and update workflow
* rename executable
* Merge pull request `#39 <https://github.com/tier4/scenario_simulator_v2/issues/39>`_ from tier4/feature/scenario_runner/user_defined_action
  Feature/scenario runner/user defined action
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/scenario_runner/user_defined_action
* Merge pull request `#38 <https://github.com/tier4/scenario_simulator_v2/issues/38>`_ from tier4/feature/launcher/refactoring_package
  Feature/launcher/refactoring package
* not testing files
* add launch
* add test set
* delete files
* arrange directory
* add files
* refactor launcher to test runner
* Contributors: Daisuke Nishimatsu, Kenji Miyake, Makoto Tokunaga, Masaya Kataoka, TaikiTanaka3, Tatsuya Yamasaki, Yamasaki Tatsuya, taikitanaka, taikitanaka3, vios-fish, wep21, yamacir-kit
