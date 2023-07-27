^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_interpreter_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#990 <https://github.com/tier4/scenario_simulator_v2/issues/990>`_ from tier4/fix/cspell_errors
* docs: use ROS 2 instead of ROS2
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* Merge remote-tracking branch 'origin/master' into clean-dicts
* Merge branch 'master' into feature/noise_delay_object
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge remote-tracking branch 'origin' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into fix/openscenario_utility/conversion
* Merge pull request `#874 <https://github.com/tier4/scenario_simulator_v2/issues/874>`_ from tier4/feature/interpreter/user-defined-value-condition
* Move a message type `UserDefinedValue` to an external package `tier4_simulation_msgs`
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Update some nodes to treat `scenario_simulator_v2_msgs` to be required dependency
* Move contents of package `user_defined_value_condition_example` to `openscenario_interpreter_example`
* Remove `openscenario_msgs` from dependency
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Shota Minami, Tatsuya Yamasaki, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, MasayaKataoka, Piotr Zyskowski, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#821 <https://github.com/tier4/scenario_simulator_v2/issues/821>`_ from tier4/feature/linelint
* add white lines at EOF
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#813 <https://github.com/tier4/scenario_simulator_v2/issues/813>`_ from tier4/fix/boost_depend
* add boost to the depends
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Fix Licence
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge pull request `#727 <https://github.com/tier4/scenario_simulator_v2/issues/727>`_ from tier4/feature/interpreter/reader
* Add new example node `uniform_distribution`
* Contributors: Daniel Marczak, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, kyabe2718, yamacir-kit
