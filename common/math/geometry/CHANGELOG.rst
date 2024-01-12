^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2023-12-21)
------------------
* Merge pull request `#1139 <https://github.com/tier4/scenario_simulator_v2/issues/1139>`_ from tier4/fix/geometry-bug-fixes
* Revert "Remove tests that do not pass"
* Merge branch 'fix/get-polygon-0-points' into fix/geometry-bug-fixes
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Fix getPolygon 0 points bug
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* Merge pull request `#1147 <https://github.com/tier4/scenario_simulator_v2/issues/1147>`_ from RobotecAI/feature/test-geometry
* Remove tests that do not pass
* Remove comments
* Merge branch 'feature/test-geometry-spline-subspline' into feature/test-geometry
* Remove empty test
* Fix CatmullRomSpline getPolygon test
* Fix linear algebra divide_zero test
* Fix HermiteCurve trajectory tests
* Merge branch 'feature/random_scenario' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* Merge remote-tracking branch 'origin/master' into feature/random_scenario
* Fix some incorrect tests
* Fix missing lambda argument
* Correct subspline collision point calculation
* Fix subspline collision point calculation
* Refactor and add getCollisionPointsIn2D function
* Correct spline tests for length estimation inaccuracy
* Merge branch 'fix/spline-max-2d-curvature' into fix/minor-bug-fixes
* Merge branch 'fix/spline-collision-bug' into fix/minor-bug-fixes
* Merge branch 'fix/remove-equal-operators' into fix/minor-bug-fixes
* Merge branch 'fix/polygon-get-min-max-value-bug' into fix/minor-bug-fixes
* Merge branch 'fix/line-segment-initialization' into fix/minor-bug-fixes
* Merge branch 'fix/intersection-vector-bug' into fix/minor-bug-fixes
* Correct CatmullRomSpline tests
* Correct literals + refactor spline & subspline tests
* Merge branch 'experimental/merge-master' into feature/test-geometry-spline-subspline
* Fix spline collision point calculation
* Fix spline maximum 2D curvature calculation
* Fix getTrajectory wrong number of points
* Fix getMinValue and getMaxValue empty vector bug
* Fix LineSegment initialization bug
* Fix intersection vector lookup after the last element
* Remove Point & Vector3 equal operators
* Correct literals and refactor + clean code in tests
* Clean test CMakeLists.txt
* Revert "Change test include directory: relative -> absolute"
* Change test include directory: relative -> absolute
* Add and correct LineSegment test cases
* Adjust LineSegment tests to new changes
* Merge remote-tracking branch 'tier4/master' into experimental/merge-master
* Adjust subspline tests to use test_utils.hpp
* Merge branch 'feature/test-geometry' into spline-subspline-tests
* Move test helper functions into common header file
* Correct exception type in Polygon.getMaxValueEmptyVector and Polygon.getMinValueEmptyVector
* Add missing CatmullRomSubspline tests
* Add missing CatmullRomSpline tests
* Correct LineSegment.initializeVectorZero test to expect error
* Refactor Polygon tests
* Refactor Intersection tests - remove loop
* Refactor Collision tests - add functions
* Refactor Transform tests - add functions
* Refactor LinearAlgebra tests - add functions
* Refactor Distance tests - add functions
* Refactor BoundingBox tests - add functions
* Add Vector3 tests
* Correct HermiteCurve tests
* Refactor CatmullRomSpline tests
* Add missing HermiteCurve tests
* Add missing Polygon tests
* Refactor HermiteCurve tests
* Refactor Polygon tests
* Add missing LineSegment tests
* Add missing Intersection tests
* Add missing Collision tests
* Add missing Transform tests
* Correct LinearAlgebra tests
* Add missing LinearAlgebra tests
* Add missing Distance tests
* Add missing BoundingBox tests
* Merge remote-tracking branch 'origin/master' into fix/rtc_command_action/continuous_execution
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge pull request `#1102 <https://github.com/tier4/scenario_simulator_v2/issues/1102>`_ from tier4/fix/wrong_distance
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/master' into fix/port_document
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* Change subtraction to assignment
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1019 <https://github.com/tier4/scenario_simulator_v2/issues/1019>`_ from tier4/feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge branch 'master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge pull request `#1095 <https://github.com/tier4/scenario_simulator_v2/issues/1095>`_ from tier4/feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* code refactor
* code refactor
* implement freespace for relative distance condition
* Init working version of DistanceCondition freespace
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into refactor/lanelet-id
* fix case
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge pull request `#1087 <https://github.com/tier4/scenario_simulator_v2/issues/1087>`_ from tier4/feature/drop_galactic_support
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* fix format
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* remove workbound for galactic
* fix typo
* add torelance
* fix getSValue function in line segment class
* add getSValue function
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* use auto
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* add member initializer
* add test case
* fix problem in total_length in CatmullRomSpline class
* use push_back
* add comment
* update comment
* add comment
* fix comment
* fix typo
* add white line
* add white line
* fix typo
* fix typo
* remove plot
* add comment
* udpate comment
* add comment
* add comment
* apply reformat
* update comment
* add gnupolot files
* update comment
* fix comment
* update if
* add comment
* add comment
* add auto scale
* add test case
* fix denormalize
* add comment
* add comment
* add comment
* use auto and ->
* use &
* use auto ->
* remove unused header
* use auto and ->
* add const
* update comment
* add description
* care edge case
* simplify code
* update comment
* update comment
* update comment
* fix typo
* add error message
* add comment
* add test case
* add comment
* add comment
* add comment
* add test case
* update test case
* remove unused function
* add autoscale to the line segment
* update description
* add description
* fix autoscale feature
* add comment
* fix typo
* enable fallback for each functions
* fix some of member function
* add white line for visibility
* add message
* add comment
* remove unused line
* add comment
* add comments
* add comment
* add comment
* add comment
* add test case
* add test case
* add docs
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* add test case
* add test case
* add test case
* fix calculate s value
* fix getLineSegments function
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* enable check collision to the point
* add functions
* simplify code
* add const
* use lambda
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Mateusz Palczuk, Michał Kiełczykowski, Paweł Lech, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge pull request `#1058 <https://github.com/tier4/scenario_simulator_v2/issues/1058>`_ from tier4/ref/RJD-553_restore_repeated_update_entity_status
* fix(common): fix missing include
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_4284' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, kyoichi-sugahara, yamacir-kit

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
