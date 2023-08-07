^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-07-26)
------------------
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#990 <https://github.com/tier4/scenario_simulator_v2/issues/990>`_ from tier4/fix/cspell_errors
* docs: use ROS 2 instead of ROS2
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/traveled_distance_as_api
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Shota Minami, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into feature/occupancy_grid_docs
* Fix rosdep dependencies
* Add intentional breaking of rosdep dependencies
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge branch 'master' into feature/remove_simple_metrics
* Merge remote-tracking branch 'origin/master' into refactor/catalog
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into doc/add_description
* Merge pull request `#862 <https://github.com/tier4/scenario_simulator_v2/issues/862>`_ from tier4/fix/rivz_depend
* change rviz depend
* Merge pull request `#857 <https://github.com/tier4/scenario_simulator_v2/issues/857>`_ from tier4/fix/remove_unused_env_variable
* remove unused depends
* remove rviz package from depends
* update depends
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into doc/6th_improvement
* Merge pull request `#837 <https://github.com/tier4/scenario_simulator_v2/issues/837>`_ from tier4/update/rviz_display
* Format files
* Add speed visualization to text marker
* Optimize quaternion description
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#816 <https://github.com/tier4/scenario_simulator_v2/issues/816>`_ from tier4/feature/geometry_lib
* fix namespavce
* modify namespace
* move directory
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* apply reformat
* Merge branch 'feature/geometry_lib' of https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* split directory
* add geometry_math package
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* apply clang format
* fix(openscenario_visualization): modify build error in humble
* more build fixes
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Fix licence
* Fix Licence
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into feature/zmqpp_vendor
* Merge pull request `#785 <https://github.com/tier4/scenario_simulator_v2/issues/785>`_ from tier4/doc/improve
* Fix broken links
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Adam Krawczyk, Daisuke Nishimatsu, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, wep21, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#744 <https://github.com/tier4/scenario_simulator_v2/issues/744>`_ from tier4/feature/remove_color_names_function
* remove functions in color_names package
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'tier4:master' into AJD-331-optimization
* Contributors: Daniel Marczak, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#686 <https://github.com/tier4/scenario_simulator_v2/issues/686>`_ from tier4/fix/warp_problem
  Fix/warp problem
* add offset visualization
* enable get - offset value
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#581 <https://github.com/tier4/scenario_simulator_v2/issues/581>`_ from Utaro-M/matsuura/feature/add-time-to-panel
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* merge fix/galactic_build
* pull master
* merge master
* Merge tier4:master
* add simulation time to panel
* update ui
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: MasayaKataoka, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Contributors: MasayaKataoka, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge commit 'ce08abe39ed83d7ec0630560d293187fbcf08b5e' into feature/add_ideal_accel_model
* Merge pull request `#619 <https://github.com/tier4/scenario_simulator_v2/issues/619>`_ from RobotecAI/AJD-254-simple_abstract_scenario_for_simple_random_testing
* random test runner
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: Masaya Kataoka, Piotr Zyskowski, dai0912th, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: kyabe2718

0.5.6 (2021-10-28)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2
* Merge pull request `#572 <https://github.com/tier4/scenario_simulator_v2/issues/572>`_ from Utaro-M/matsuura/feature/add-icon-to-panel
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/galactic_docker_image
* Merge pull request `#575 <https://github.com/tier4/scenario_simulator_v2/issues/575>`_ from tier4/fix/typo
* fix typo detected from https://github.com/tier4/scenario_simulator_v2/runs/3923309766?check_suite_focus=true
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* fix spell
* add icon to context panel
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* Merge pull request `#569 <https://github.com/tier4/scenario_simulator_v2/issues/569>`_ from Utaro-M/matsuura/feature/fix_context_panel
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* fix format
* fix context panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------

0.5.4 (2021-10-13)
------------------
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Merge pull request `#555 <https://github.com/tier4/scenario_simulator_v2/issues/555>`_ from Utaro-M/matsuura/feature/context_panel
* fix format
* add context panel
* Contributors: Masaya Kataoka, Yutaro Matsuura, yamacir-kit

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
* modify visualiza position
* enable visualize position
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* fix typo of goal_pose
* Revert "Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel"
* Merge branch 'master' into fix/offset_calculation
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* Merge pull request `#458 <https://github.com/tier4/scenario_simulator_v2/issues/458>`_ from Utaro-M/add-goalpose
* add new test case source
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix format
* fix typo
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* add goal_pose arrow marker
* add goalpose arrow
* visualization save
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* avoid clash when visualizing context
* add spin thread
* stop selection after select topic
* update panel
* use std::chrono and thread
* add topic list
* enable visualize icon
* enable add panel to the rviz
* configure directory
* enable pass compile
* apply reformat
* add depends
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura

0.4.5 (2021-08-30)
------------------
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: MasayaKataoka, Wojciech Jaworski, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge pull request `#432 <https://github.com/tier4/scenario_simulator_v2/issues/432>`_ from tier4/fix/suppress_warnings
* fix rclcpp::Duration deprecated
* add Werror
* Merge remote-tracking branch 'origin/master' into namespace
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* Merge branch 'master' into namespace
* Contributors: Hiroki OTA, Masaya Kataoka, kyabe2718, yamacir-kit

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
* Merge branch 'master' into traffic_signal_actions
* Contributors: kyabe2718

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
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* Fix typos and grammars in docs and comments
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Contributors: Kazuki Miyahara, Masaya Kataoka

0.0.1 (2021-05-12)
------------------
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* use clang_format
* apply reformat
* rename simulation_api package
* Merge branch 'master' into feature/test-runner/autoware.launch.xml
* Merge pull request `#231 <https://github.com/tier4/scenario_simulator_v2/issues/231>`_ from tier4/feature/add_contributing_md
  Feature/fix_licence_problems
* modify package.xml
* Merge branch 'master' into doc/zeromq
* Merge pull request `#225 <https://github.com/tier4/scenario_simulator_v2/issues/225>`_ from tier4/feature/support-autoware.iv-9
  Feature/support autoware.iv 9
* Fix raycasting
* Merge pull request `#160 <https://github.com/tier4/scenario_simulator_v2/issues/160>`_ from tier4/feature/visualize_other_action_stop_position
  Feature/visualize other action stop position
* update obstacle marker position
* updat rviz
* updat ebbox size
* Merge pull request `#159 <https://github.com/tier4/scenario_simulator_v2/issues/159>`_ from tier4/feature/obstacle_visualization
  Feature/obstacle visualization
* visualize stop position
* remove rviz error
* modify obstacle message
* update rviz
* add getNormalVector function
* Merge pull request `#149 <https://github.com/tier4/scenario_simulator_v2/issues/149>`_ from tier4/feature/foxy
  Feature/foxy
* Merge pull request `#151 <https://github.com/tier4/scenario_simulator_v2/issues/151>`_ from tier4/fix/go_wrong_way
  Fix/go wrong way
* fix trajectory marker
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge pull request `#150 <https://github.com/tier4/scenario_simulator_v2/issues/150>`_ from tier4/feature/entity_waypoint
  Feature/entity waypoint
* Reformat openscenario/*
* add marker to the other actions
* configure marker parameters
* apply reformat
* configure origin
* change default parameters
* configure message
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge branch 'master' into refactor/scenario-test-runner-2
* Merge pull request `#147 <https://github.com/tier4/scenario_simulator_v2/issues/147>`_ from tier4/feature/remove_entity_status
  Feature/remove entity status
* enable spawn in relative pose
* Merge branch 'master' into feature/ordered-xosc
* Merge pull request `#140 <https://github.com/tier4/scenario_simulator_v2/issues/140>`_ from tier4/feature/lanlet_pose
  Feature/lanlet pose
* add message types
* Merge branch 'master' into doc/test_runner
* Merge pull request `#117 <https://github.com/tier4/scenario_simulator_v2/issues/117>`_ from tier4/document/openscenario_interpreter
  Document/openscenario interpreter
* Merge remote-tracking branch 'origin/master' into doc/test_runner
* Fix company name
* Merge branch 'master' into feature/awapi_adapter/add_info
* Merge pull request `#97 <https://github.com/tier4/scenario_simulator_v2/issues/97>`_ from tier4/feature/doxybook
  Feature/doxybook
* remove add_document command from cmake
* remove document generator from depends
* add doxybook and doxygen config file
* Merge remote-tracking branch 'origin/master' into feature/awapi_adapter/add_info
* Merge branch 'master' into feature/awapi_adapter/vehicle_info
* Merge pull request `#93 <https://github.com/tier4/scenario_simulator_v2/issues/93>`_ from tier4/fix/copyright
  update copyright
* update copyright
* Merge remote-tracking branch 'origin/master' into feature/awapi_adapter/lib
* Merge branch 'master' into feature/awapi_awauto_adapter
* Merge pull request `#86 <https://github.com/tier4/scenario_simulator_v2/issues/86>`_ from tier4/feature/ego_vehicle
  Feature/ego vehicle
* add doxygen comment
* update documentation
* fix compile errors
* enable subscribe vehicle command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/autoware_moc
* Merge branch 'master' into feature/awapi_awauto_adapter
* Merge pull request `#84 <https://github.com/tier4/scenario_simulator_v2/issues/84>`_ from tier4/documentation/simulation_api
  Documentation/simulation api
* add documentation about opnscenario visualization component
* restore document generator
* Merge branch 'master' into feature/test_runner/add_implementation_details
* Merge branch 'master' into refactor/converter
* Merge pull request `#69 <https://github.com/tier4/scenario_simulator_v2/issues/69>`_ from tier4/feature/vnc
  Feature/vnc
* add image
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/error-handling
* Merge pull request `#61 <https://github.com/tier4/scenario_simulator_v2/issues/61>`_ from tier4/feature/stop_at_stopline
  Feature/stop at stopline
* Merge pull request `#59 <https://github.com/tier4/scenario_simulator_v2/issues/59>`_ from tier4/feature/enhance_visualization
  Feature/enhance visualization
* enable transfrom base link frame
* enable pass colcon test
* change text position
* enbale visualize action
* update rviz file
* change marker color
* add text action marker
* add [[maybe_unused]]
* apply reformat
* enable display wireframe bbox
* remove whitespace
* enable pass colcon test
* apply reformat
* enable delete unused entity
* add brake line
* enable visualize marker
* enable publish delete marker
* apply reformat and modify launch files
* add visualization node
* add lifecycle methods
* register as component node
* add OpenScenarioVisuzlization class
* add openscenario visualization directory
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, taikitanaka, taikitanaka3, yamacir-kit
