^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-07-26)
------------------
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* fix(traffic_sim): fix interface update
* feat(traffic_sim): ensure max_jerk as variable in entity_base
* fix(traffic_sim): fix dynamic constraints in actioons
* ref(traffic_sim):  out_of_range  only for npc vehicles, add tolerance
* ref(traffic_sim): append out_of_range to job_list\_
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge branch 'master' into import/universe-2437
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge branch 'master' into feature/get_lateral_distance
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge pull request `#946 <https://github.com/tier4/scenario_simulator_v2/issues/946>`_ from tier4/fix/stop_behavior
* update stopping behavior
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* chore(ci): delete interface change
* chore(ci): test interface breaking
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
* add white line at EOF
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* modify default value
* add default value
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* add resetDynamicConstraints(); function
* add new field
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Shota Minami, Tatsuya Yamasaki, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge pull request `#900 <https://github.com/tier4/scenario_simulator_v2/issues/900>`_ from tier4/feature/traffic_simulator/behavior-parameter
* Increase the default maximum acceleration of `DynamicConstraints`
* Updates `setBehaviorParameter` to clamp the given value with maximum performance
* Add new message type `traffic_simulator_msgs::msg::DynamicConstraints`
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* Merge branch 'master' into fix/interpreter/custom_command_action
* Add some max constraints to `msg::BehaviorParameter`
* Add `max\_(acceleration|deceleration)_rate` to `traffic_simulator_msgs::msg::Performance`
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#898 <https://github.com/tier4/scenario_simulator_v2/issues/898>`_ from tier4/feature/interpreter/speed-profile-action
* Add test scenario `LongitudinalAction.SpeedProfileAction`
* Rename `DriverModel` to `BehaviorParameter`
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#821 <https://github.com/tier4/scenario_simulator_v2/issues/821>`_ from tier4/feature/linelint
* fix line error
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Daniel Marczak, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* rename data field
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* rename to subtype
* add white line at EOL
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* add entity semantics to the member variable
* add EntitySemantics data type
* Contributors: Daniel Marczak, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Contributors: yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* merge fix/galactic_build
* pull master
* merge master
* Merge tier4:master
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* remove ROS message
* add requestSpeedChange API
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: MasayaKataoka, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Contributors: MasayaKataoka

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: Piotr Zyskowski, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge branch 'master' into feature/interpreter/catalog
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
* Contributors: MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------

0.5.4 (2021-10-13)
------------------
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Contributors: yamacir-kit

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
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Contributors: yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge pull request `#507 <https://github.com/tier4/scenario_simulator_v2/issues/507>`_ from tier4/feature/add_scenario
* update lane assing logic for pedestrian
* Merge branch 'master' into fix/offset_calculation
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* Merge pull request `#458 <https://github.com/tier4/scenario_simulator_v2/issues/458>`_ from Utaro-M/add-goalpose
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#485 <https://github.com/tier4/scenario_simulator_v2/issues/485>`_ from tier4/feature/test_simulation_interface
* Merge branch 'master' into add-goalpose
* remove unused message
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' into add-goalpose
* fix typo
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* add getGoalposes()
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura

0.4.5 (2021-08-30)
------------------
* Merge pull request `#457 <https://github.com/tier4/scenario_simulator_v2/issues/457>`_ from tier4/feature/math_test
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* remove unused message
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* remove unused message
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Masaya Kataoka, MasayaKataoka, Wojciech Jaworski, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, kyabe2718

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

0.4.0 (2021-07-27)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Contributors: yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* add MiscObjectParameters message type
* add Misc Object type
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718

0.2.0 (2021-06-24)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge pull request `#344 <https://github.com/tier4/scenario_simulator_v2/issues/344>`_ from tier4/feature/interpreter/context
* Add new package 'openscenario_interpreter_msgs'
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge remote-tracking branch 'origin/master' into fix/traffic-simulator/simulation-model-2
* Merge pull request `#349 <https://github.com/tier4/scenario_simulator_v2/issues/349>`_ from tier4/fix/typos-in-openscenario-dir
* Fix typos in the openscenario directory
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Contributors: Kazuki Miyahara, Masaya Kataoka, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Merge branch 'master' into fix/misc-problems
* Merge pull request `#238 <https://github.com/tier4/scenario_simulator_v2/issues/238>`_ from tier4/feature/interpreter/vehicle/base_link-offset
  Remove member function `API::spawn` receives XML strings.
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* Update type Performance, Axles and Axle to support cast operator
* use clang_format
* Add cast operator for geometry_msgs::msg::Point to Center type
* Move VehicleCategory's stream I/O operators into .cpp file
* move some headers
* move some logics
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#195 <https://github.com/tier4/scenario_simulator_v2/issues/195>`_ from tier4/feature/use_protobuf_in_spawn
  Feature/use protobuf in spawn
* remove is_ego parameter
* fix typo
* add parameter message
* add vehicle parameters message
* add Property message
* add axles message
* add message
* add performance message
* Merge branch 'master' into feature/support-autoware.iv
* Merge branch 'master' into feature/awapi-accessor/non-awapi-topics
* Merge pull request `#188 <https://github.com/tier4/scenario_simulator_v2/issues/188>`_ from tier4/feature/idiot_npc
  Feature/idiot npc
* update driver model
* Merge pull request `#162 <https://github.com/tier4/scenario_simulator_v2/issues/162>`_ from tier4/feature/driver_model
  Feature/driver model
* add comment
* add driver model message
* Merge pull request `#159 <https://github.com/tier4/scenario_simulator_v2/issues/159>`_ from tier4/feature/obstacle_visualization
  Feature/obstacle visualization
* enable get obstacle via entity manager
* modify obstacle message
* modify obstacle message
* add obstacle output port
* add obstacle type
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge pull request `#150 <https://github.com/tier4/scenario_simulator_v2/issues/150>`_ from tier4/feature/entity_waypoint
  Feature/entity waypoint
* add calculate obstacles function
* configure message
* enable calculate waypoint while lane follow
* add waypoint message type
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge branch 'master' into refactor/scenario-test-runner-2
* add trajectory message type
* add curve message
* Merge pull request `#147 <https://github.com/tier4/scenario_simulator_v2/issues/147>`_ from tier4/feature/remove_entity_status
  Feature/remove entity status
* Merge branch 'feature/remove_entity_status' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* reduce compile errors
* add EntityType message
* Merge branch 'master' into feature/ordered-xosc
* Merge pull request `#140 <https://github.com/tier4/scenario_simulator_v2/issues/140>`_ from tier4/feature/lanlet_pose
  Feature/lanlet pose
* modify message
* remove twist/accel from entity status
* add message types
* add LaneletPose message
* Merge branch 'master' into feature/test_runner/add_implementation_details
* Merge branch 'master' into feature/remove-github-access-token
* Merge branch 'master' into refactor/converter
* Merge pull request `#65 <https://github.com/tier4/scenario_simulator_v2/issues/65>`_ from tier4/feature/yield
  Feature/yield
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/error-handling
* change lanlet id type from int to int64
* Merge pull request `#59 <https://github.com/tier4/scenario_simulator_v2/issues/59>`_ from tier4/feature/enhance_visualization
  Feature/enhance visualization
* add text action marker
* enable publish bounding box
* add openscenario visualization directory
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, yamacir-kit
