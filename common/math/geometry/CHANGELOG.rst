^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-07-26)
------------------
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#906 <https://github.com/tier4/scenario_simulator_v2/issues/906>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Move some vector3 related functions into package `geometry`
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into fix/hdmap_utils/get_stop_lines
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/rtc_custom_command_action
* Merge pull request `#996 <https://github.com/tier4/scenario_simulator_v2/issues/996>`_ from tier4/fix/get_s_value
* add const
* add const
* add const
* fix indent
* add const
* modify return type
* rename to isApproximatelyEqualTo
* sort test_depend
* Update common/math/geometry/src/spline/hermite_curve.cpp
* fix indent
* sort package.xml
* reduce comment lines
* update comment
* simplify code
* format
* move q3 position
* remove a2
* use std::apply
* update code
* add comment
* modify exception text
* add comment
* clanup code
* remove unused value
* add isEqual function
* add comment
* cleanup code
* add filterByRange function
* apply format
* rename functions
* add const to some values, add comments
* fix comment
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* rename test case
* fix comment
* add comment for test cases
* update comment
* update comment
* update comment
* update comment
* update comment
* update comment
* sort functions
* add comment
* add test case
* format
* fix compile error
* add testcase
* remove unused if
* fix comment
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* use plural
* use singular
* use singular
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* fix test case result
* update comment
* update comment
* update comment
* reduce line
* Unify terms (root and solution) that refer to the same object used in variable names and comments
* update comment
* add the
* add comment
* modify comment
* update comment
* remove comment
* fix compile error
* use auto
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* add description and update dictionary
* use push_back
* use push_back and add comment
* add description
* modify if
* fix comment
* fix test case
* add comment
* replace to stl function
* add note comments for test cases
* fix word choice
* add . at the end of the sentence
* fix description
* fix description
* fix gramtical errors
* Unified formatting
* fix typo
* modify test cases
* care ax*b = 0 (a=0, b≠0) case
* use math::geometry::PolynominalSolver::tolerance
* fix unit test case
* add ,
* describe why the problem happens more deeply
* add comment for test case description
* add comment
* throw errors when any value of min_value~max_value will be the solution.
* throw error when the case was not computable
* use push_back with scalar type
* remove unclear comment
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* add comment
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* remove debug line
* remove unused line
* consider tolerance when the solver return value
* add test case
* add test case
* use constexpr
* remove header
* remove header
* remove unused header file
* remove unused depend
* remove debug lines
* rename to torelance
* add epsilon
* add test case
* add archive
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki, hrjp, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#993 <https://github.com/tier4/scenario_simulator_v2/issues/993>`_ from tier4/fix/add_const
* add const
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#894 <https://github.com/tier4/scenario_simulator_v2/issues/894>`_ from tier4/fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge branch 'master' into feature/noise_lost_object
* Merge pull request `#951 <https://github.com/tier4/scenario_simulator_v2/issues/951>`_ from tier4/fix/warnings
* remove C++ warnings
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
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
* split functions
* Add missing headers
* Format
* Replace boost::optional with std::optional
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into fix_wrong_merge
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Shota Minami, Tatsuya Yamasaki, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/fix/ci_error' into feature/start_npc_logic_api
* Merge branch 'master' into feature/occupancy_grid_docs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge pull request `#870 <https://github.com/tier4/scenario_simulator_v2/issues/870>`_ from tier4/prepare/release_0.6.6
* modify version
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge pull request `#847 <https://github.com/tier4/scenario_simulator_v2/issues/847>`_ from tier4/feature/value_constraint
* Replace "Tier IV" with "TIER IV"
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'tier/master' into fix/obstacle_detection_raycaster
* Merge pull request `#836 <https://github.com/tier4/scenario_simulator_v2/issues/836>`_ from tier4/fix/trajectory_offset
* fix problems in trajectory offset
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#816 <https://github.com/tier4/scenario_simulator_v2/issues/816>`_ from tier4/feature/geometry_lib
* fix include guard
* fix namespace
* fix namespavce
* modify namespace
* modify namespace
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* move directory
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, kyabe2718, yamacir-kit
