^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simulation_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* Revert removed EXPECT_TRUE()
* typo fix, unnecessary test removed
* missing tests
* disabled newer traffic lights shapes
* clang format
* traffic lights toMsg conversion
* traffic lights interface change; test fix
* brought back working version with SSS (break working with AWSIM)
* working changes
* commended logs
* zmq debug
* moved EES to SSS
* lanelet2 map passing via zmq
* entity status zmq update
* pedestrian and misc object models passed
* added unique key, pose and initial speed to the spawn vehicle
* map to keep entity status in sss; zmq entity update takes one entity at a time
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* further removed updatesensorframe from zmq interface
* merged UpdateSensorFrame into UpdateFrameRequest
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* initialize changed
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#990 <https://github.com/tier4/scenario_simulator_v2/issues/990>`_ from tier4/fix/cspell_errors
* docs: use ROS 2 instead of ROS2
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge branch 'master' into feature/interpreter/environment
* Merge pull request `#986 <https://github.com/tier4/scenario_simulator_v2/issues/986>`_ from tier4/feature/interpreter/publishing-delay
* Merge branch 'master' into feature/interpreter/model3d-field
* Merge remote-tracking branch 'origin/master' into feature/interpreter/publishing-delay
* Lipsticks
* Merge branch 'master' into fix/cleanup_code
* Merge branch 'master' into feature/interpreter/environment
* Merge pull request `#981 <https://github.com/tier4/scenario_simulator_v2/issues/981>`_ from RobotecAI/ref/AJD-697_improve_port_management_zmq
* ref(various): add EOF, update ReleaseNotes
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* Revert "feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit"
* Merge remote-tracking branch 'origin/master' into clean-dicts
* ref(sim_interface): apply ament clang reformat
* ref(sim_interface):  reduce number of zmq ports to 1
* Merge branch 'master' into feature/interpreter/model3d-field
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#964 <https://github.com/tier4/scenario_simulator_v2/issues/964>`_ from tier4/feature/noise_delay_object
* fix conflict
* add param
* Merge branch 'master' into feature/noise_delay_object
* feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into fix/get-unique-route-lanelets
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#967 <https://github.com/tier4/scenario_simulator_v2/issues/967>`_ from RobotecAI/fix/AJD-655-terminates-sigint
* Merge pull request `#932 <https://github.com/tier4/scenario_simulator_v2/issues/932>`_ from tier4/feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* refactor: apply ament_clang_format
* fix(os_interp): fix abort caused by ~Interpreter
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin' into fix/getting_next_lanelet
* Merge pull request `#958 <https://github.com/tier4/scenario_simulator_v2/issues/958>`_ from tier4/feature/noise_lost_object
* added param probability of lost recognition
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* Update `MultiServer` to be monitored by `status_monitor`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#914 <https://github.com/tier4/scenario_simulator_v2/issues/914>`_ from tier4/feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#909 <https://github.com/tier4/scenario_simulator_v2/issues/909>`_ from tier4/feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* update proto
* add simple noise generator
* fix PERFORMANCE_EQ macro
* change default value
* update message proto
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Shota Minami, Tatsuya Yamasaki, f0reachARR, hrjp, kyoichi sugahara, kyoichi-sugahara, yamacir-kit, ぐるぐる

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Contributors: Kotaro Yoshimoto, MasayaKataoka, Piotr Zyskowski, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge pull request `#805 <https://github.com/tier4/scenario_simulator_v2/issues/805>`_ from tier4/doc/4th-improvement
* Add units to configuration document
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#813 <https://github.com/tier4/scenario_simulator_v2/issues/813>`_ from tier4/fix/boost_depend
* use boost
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pull_over_metrics
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge pull request `#797 <https://github.com/tier4/scenario_simulator_v2/issues/797>`_ from tier4/feature/occupancy_grid_sensor
* Merge pull request `#778 <https://github.com/tier4/scenario_simulator_v2/issues/778>`_ from tier4/feature/zmqpp_vendor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Fix Licence
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* fix problems in server
* fix compile errors
* fix reorder warnings
* add occupancy grid sensor API to the server
* add client impl
* update proto
* modify CMakeLists.txt
* modify depends
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/job_system
* Merge pull request `#690 <https://github.com/tier4/scenario_simulator_v2/issues/690>`_ from RobotecAI/AJD-331-make-zmq-client-work-through-network
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* shutdown_flag -> is_running
* ament_clang_format
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#729 <https://github.com/tier4/scenario_simulator_v2/issues/729>`_ from tier4/feature/ignore_raycast_result
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* update error message
* rename data field
* enable filter by range
* update proto
* rename args
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* use switch in conversion
* rename to subtype
* Merge branch 'feature/semantics' of https://github.com/tier4/scenario_simulator_v2 into feature/semantics
* remove category
* fix typo and remove debug line
* add disconect() to ~Interpreter(). stop zeromq call if shut down.
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* fix error
* add test code
* add semantics conversion function
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* modify proto file
* add EntityTypeProto
* enable filter by range
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* revert some changes from option 2
* change to option 1 (simulator host as a launch parameter)
* fix clang format
* internal review fixes
* zmq client can connect through the network
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#683 <https://github.com/tier4/scenario_simulator_v2/issues/683>`_ from tier4/feature/zeromq_multi_client
  Feature/zeromq multi client
* add EOL white line
* remove unused files
* use close function
* use multi client class
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#673 <https://github.com/tier4/scenario_simulator_v2/issues/673>`_ from tier4/fix/universe/traffic-signal
  Fix `toProto` for `autoware_auto_perception_msgs::msg::TrafficSignal`
* Add unit test for conversion TrafficSignal => TrafficLightState
* Fix `toProto` for `autoware_auto_perception_msgs::msg::TrafficSignal`
* Add missing new protobuf files
* Replace `VehicleCommand` with `AckermannControlCommand` and `GearCommand`
* Remove package `autoware_perception_msgs`
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

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
* Add `autoware_control_msgs` to dependency of `simulation_interface`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: Masaya Kataoka, MasayaKataoka, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Rename `LiDAR` to `Lidar`
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Update class `SensorSimulation` to choice topic name and type based on Autoware's architecture type
* Rename protobuf message `TrafficLightState` field `lights` to `lamp_states`
* Remove `autoware_auto_control_msgs.proto`
* Remove unused protobuf <=> message conversions
* Restore virtual function `EntityBase::getVehicleCommand`
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* Comment-out some tests and Remove protobuf type `GearCommand`, `GearReport`
* detected object -> predicted object, and apply ament_clang_format
* Remove member function `getVehicleCommand` from Vehicle type entity
* dealt with test conversion, and fix proto of deprecated vehicle command
* Add some protobuf <=> ROS2 message conversions
* fix toProt/Msgs of shift
* fix topic type name
* control command -> ackermann control command
* use auto_msgs for dynamic objects
* use auto_msgs for traffic lights
* Contributors: MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: Piotr Zyskowski, yamacir-kit

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
* Rename `openscenario_msgs.proto` to `traffic_simulator_msgs.proto`
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Contributors: MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------

0.5.4 (2021-10-13)
------------------
* Merge pull request `#557 <https://github.com/tier4/scenario_simulator_v2/issues/557>`_ from tier4/revert/pr_544
* Revert "Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status"
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------

0.5.2 (2021-10-06)
------------------
* Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status
* remove boost::none status in traffic_simulator
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#530 <https://github.com/tier4/scenario_simulator_v2/issues/530>`_ from RobotecAI/traffic_lights
* Typos fix
* Clang formatting and conversions test for traffic light
* ZMQ api for traffic lights
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge pull request `#516 <https://github.com/tier4/scenario_simulator_v2/issues/516>`_ from tier4/fix/zmq-poll-arg
* Fix the argument of zmqpp::poller::poll()
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Contributors: Kazuki Miyahara, Masaya Kataoka, Piotr Jaroszek, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* gix some typo
* fix typo of definition
* fix typo of definition
* fix typo of definicion
* fix typo of action
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#485 <https://github.com/tier4/scenario_simulator_v2/issues/485>`_ from tier4/feature/test_simulation_interface
* change character
* Merge branch 'master' into add-goalpose
* fix typo
* fix typo
* add test cases for enum value
* change error type
* add testcase for clock conversion
* enable ignore unreachable lines
* initialize string value
* remove unused enum value
* modify expect macro for vehicle command
* add EXPECT_VEHICLE_EQ macro
* add test cases for toMsg function in Entity Status message
* add test cases for entity status
* add macros for lanelet pose
* add EXPECT_ACTION_STATUS_EQ macro
* add test cases for pedestrian and misc object parameters
* add test case for vehicle parameters
* fix include
* add test case for bounding box
* add some comments
* split files
* add EXPECT_AXLE_EQ and EXPECT_AXLES_EQ macro
* add EXPECT_TWIST_EQ, EXPECT_ACCEL_EQ macro
* add macros
* apply reformat
* remove warning
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* add test case for entity type conversion
* add test case for converting misc object type
* add test case for lanelet pose
* add test cases
* add test case
* enable convert pedestrian parameters
* remove default label
* remove unused example
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura

0.4.5 (2021-08-30)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: MasayaKataoka, Wojciech Jaworski, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge pull request `#448 <https://github.com/tier4/scenario_simulator_v2/issues/448>`_ from tier4/feature/add_cpp_scenarios
* add call thread::join in destructor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge remote-tracking branch 'origin/master' into namespace
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* Merge branch 'master' into namespace
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------

0.4.1 (2021-07-30)
------------------
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Contributors: yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#411 <https://github.com/tier4/scenario_simulator_v2/issues/411>`_ from tier4/doc/run_scenario_test_with_docker
* fix typo
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* enable send SpawnMiscObjectEntityRequest to the sensor simulator
* update zeromq server
* fix test code in tntity type
* add test cases
* add toProto toMsg functions
* add MiscObjectParameters message type
* add Misc Object type
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718

0.2.0 (2021-06-24)
------------------
* Merge pull request `#357 <https://github.com/tier4/scenario_simulator_v2/issues/357>`_ from tier4/feature/send_ego_command
* fix typo
* fix indent
* fix indent
* fix indend
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* apply clang-format
* add proto files
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Contributors: kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/get_vehicle_cmd
* Merge pull request `#354 <https://github.com/tier4/scenario_simulator_v2/issues/354>`_ from tier4/fix/typos-misc
* Fix typos in docs / mock / simulation/ test_runner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* Fix typos and grammars in docs and comments
* Merge pull request `#306 <https://github.com/tier4/scenario_simulator_v2/issues/306>`_ from tier4/feature/use_common_exception
* Remove trailing semicolon from macro definition
* remove protobuf conversion error
* remove unused macro
* remove XML parameter error
* remove SimulationClockError
* remove Execution Error
* Merge pull request `#304 <https://github.com/tier4/scenario_simulator_v2/issues/304>`_ from tier4/feature/synchronize_clock
* modify API schema
* enable convert clock topic
* add Clock message definition
* add messages
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
* remove flake8 check
* add new line for the block
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* use clang_format
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* apply reformat
* Merge pull request `#252 <https://github.com/tier4/scenario_simulator_v2/issues/252>`_ from tier4/feature/publish_proto_doc
  Feature/publish proto doc
* enable pass textlint
* udpate proto
* modify case
* enable generate link
* add document about openscenario_msgs.proto
* add document comment
* update docs
* fix case
* remove unusd proto files
* Merge branch 'master' into feature/test-runner/autoware.launch.xml
* Merge pull request `#231 <https://github.com/tier4/scenario_simulator_v2/issues/231>`_ from tier4/feature/add_contributing_md
  Feature/fix_licence_problems
* fix e-mail
* modify package.xml
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-8
* Merge pull request `#219 <https://github.com/tier4/scenario_simulator_v2/issues/219>`_ from tier4/feature/remove_xmlrpc
  Feature/remove xmlrpc
* Merge branch 'master' into feature/remove_xmlrpc
* Merge pull request `#217 <https://github.com/tier4/scenario_simulator_v2/issues/217>`_ from tier4/feature/zeromq_integration
  Feature/zeromq integration
* add default
* enable pass linter
* remove stop watch
* add all methods
* add lidar callback
* add despawn entity gunction
* add spawn pedestrian func
* add spawn vehicle entity
* add soe callbacks
* add multi server
* remove debuf lines
* add stop watch
* add stopwatch class
* update constants
* remove unused files
* check transport repeated field
* remove xmlrclcpp
* enable pass cpplint
* apply reformat
* add port configuration
* add constructor to the server
* add hostname enum
* add constructor
* Merge branch 'master' into feature/support-autoware.iv-6
* Merge pull request `#216 <https://github.com/tier4/scenario_simulator_v2/issues/216>`_ from tier4/fix/test_in_simulation_interface
  Fix/test in simulation interface
* enable pass flake8
* apply reformat
* Merge branch 'master' into feature/support-autoware.iv-6
* enable call client
* add debug line
* add debug line
* add launch file
* Merge branch 'master' into feature/support-autoware.iv-4
* Merge pull request `#207 <https://github.com/tier4/scenario_simulator_v2/issues/207>`_ from tier4/fix/xmlrpc_connection_lost
  Fix/xmlrpc connection lost
* enable set status
* remove unused fields
* fix scenario
* add debug lines
* remove unused lines
* remove multicall
* remove warnings
* remove multicall
* Merge pull request `#202 <https://github.com/tier4/scenario_simulator_v2/issues/202>`_ from tier4/feature/get_waypoint_from_autoware
  Feature/get waypoint from autoware
* add proto
* Merge branch 'feature/publish_obstacle' of https://github.com/tier4/scenario_simulator.auto into feature/get_waypoint_from_autoware
* modify include gurad
* rename packages
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, yamacir-kit
