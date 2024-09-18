^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2024-04-18)
-------------------
* Merge branch 'master' into refactor/drop_workflow
* Merge remote-tracking branch 'origin/master' into refactor/drop_workflow
  # Conflicts:
  #	test_runner/scenario_test_runner/config/workflow_example.yaml
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Contributors: Kotaro Yoshimoto

2.5.0 (2024-07-08)
------------------
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Contributors: Masaya Kataoka

4.2.8 (2024-09-18)
------------------

4.2.7 (2024-09-13)
------------------

4.2.6 (2024-09-13)
------------------
* Merge branch 'master' into RJD-1197/pose_module
* Contributors: Masaya Kataoka

4.2.5 (2024-09-12)
------------------

4.2.4 (2024-09-12)
------------------

4.2.3 (2024-09-11)
------------------

4.2.2 (2024-09-10)
------------------
* Merge pull request `#1341 <https://github.com/tier4/scenario_simulator_v2/issues/1341>`_ from tier4/RJD-1278/geometry-update
  Rjd 1278/geometry update
* Merge branch 'master' into RJD-1278/geometry-update
* updates after merge
* update testcase
* remove empty line
* isIntersect2D_collinear
* refactor toPolygon2D tests
* add comments
* bounding_box clean up
* clean up vector3
* rename tests in HermiteCurve
* rename tests in CatmullRomSpline
* quaternion operators
* tune down numbers
* sort tests, rm old duplicate
* getIntersection2D function
* getIntersection2DSValue and isIntersect2D functions
* getPose, getPoint refactor
* add a proper structure to the test files
* Merge branch 'master' into RJD-1278/geometry-update
* Merge branch 'master' into RJD-1278/geometry-update
* update testcase
* remove empty line
* isIntersect2D_collinear
* Merge branch 'master' into RJD-1278/geometry-update
* refactor toPolygon2D tests
* add comments
* bounding_box clean up
* clean up vector3
* rename tests in HermiteCurve
* rename tests in CatmullRomSpline
* quaternion operators
* tune down numbers
* sort tests, rm old duplicate
* getIntersection2D function
* getIntersection2DSValue and isIntersect2D functions
* getPose, getPoint refactor
* add a proper structure to the test files
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.2.1 (2024-09-10)
------------------

4.2.0 (2024-09-09)
------------------

4.1.1 (2024-09-03)
------------------
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge remote-tracking branch 'origin/master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into doc/RJD-1273-add-realtime-factor-doc
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto

4.1.0 (2024-09-03)
------------------
* Merge pull request `#1353 <https://github.com/tier4/scenario_simulator_v2/issues/1353>`_ from tier4/RJD-1278/fix-line-segment
  Rjd 1278/fix line segment
* Merge branch 'master' into RJD-1278/fix-line-segment
* make const members public
* Merge branch 'master' into RJD-1278/fix-line-segment
* Merge branch 'master' into RJD-1278/fix-line-segment
* remove{}
* expand on the note, add else to if-stmts
* note
* add else to if statements
* rename getSlope, add consts
* remove unnecessary lambda
* simplify denormalize logic
* use has_value
* rename getIntersection2DSValue, minor logical fixes regarding 2D vs 3D
* LineSegment 2D vs 3D indistinction fixes
* return const& and remove implicit conversions
* vector fields for LineSegment class, cleanup
* use isInBounds function
* combine 2 PR, apply review requests
* Merge branch 'RJD-1278/fix-1344-getIntersection2DSValue' of github.com:tier4/scenario_simulator_v2 into RJD-1278/fix-line-segment
* Merge branch 'master' into RJD-1278/fix-1344-getIntersection2DSValue
* Merge branch 'master' into RJD-1278/fix-1343-isIntersect2D
* isIntesect2D initial solution
* 1344 fix initial solution
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.0.4 (2024-09-02)
------------------
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Contributors: Masaya Kataoka, SzymonParapura

4.0.3 (2024-08-29)
------------------
* Merge remote-tracking branch 'origin/master' into RJD-1056-remove-npc-logic-started
* Merge branch 'RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'RJD-1056-remove-npc-logic-started' into RJD-1057-base
* Merge branch 'RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'tier4/RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'origin/RJD-1056-remove-npc-logic-started' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'origin/ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Contributors: DMoszynski, Dawid Moszynski, Mateusz Palczuk

4.0.2 (2024-08-28)
------------------
* Merge branch 'master' into RJD-1056-remove-current-time-step-time
* Merge branch 'master' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge remote-tracking branch 'origin/ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Mateusz Palczuk

4.0.1 (2024-08-28)
------------------
* Merge branch 'master' into fix/follow_trajectory
* Merge branch 'master' into fix/follow_trajectory
* Merge remote-tracking branch 'origin' into fix/follow_trajectory
* Contributors: Masaya Kataoka

4.0.0 (2024-08-27)
------------------
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge remote-tracking branch 'origin/ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge remote-tracking branch 'origin/ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Mateusz Palczuk, Tatsuya Yamasaki

3.5.5 (2024-08-27)
------------------
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* Contributors: Kotaro Yoshimoto

3.5.4 (2024-08-26)
------------------
* Merge branch 'master' into feature/use_workflow_dispatch_in_docker_build
* Merge branch 'master' into feature/use_workflow_dispatch_in_docker_build
* Merge remote-tracking branch 'origin/master' into feature/use_workflow_dispatch_in_docker_build
* Merge remote-tracking branch 'origin/master' into feature/trigger_docker_build_by_tag
* Contributors: Masaya Kataoka

3.5.3 (2024-08-26)
------------------
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Contributors: Michał Ciasnocha

3.5.2 (2024-08-23)
------------------
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into fix/interpreter/user-defined-value-condition
* Contributors: Tatsuya Yamasaki, yamacir-kit

3.5.1 (2024-08-22)
------------------
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Contributors: Dawid Moszyński, Tatsuya Yamasaki

3.5.0 (2024-08-21)
------------------
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/relative-clearance-condition' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

3.4.4 (2024-08-20)
------------------

3.4.3 (2024-08-19)
------------------

3.4.2 (2024-08-05)
------------------
* Merge commit 'c1cab6eb1ece2df58982f50a78fef5a5ecaa7234' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Contributors: Masaya Kataoka, SzymonParapura, koki suzuki

3.4.1 (2024-07-30)
------------------
* Merge branch 'master' into doc/open_scenario_support
* Contributors: Tatsuya Yamasaki

3.4.0 (2024-07-26)
------------------

3.3.0 (2024-07-23)
------------------
* Merge branch 'master' into feature/interpreter/entity_selection
* Merge branch 'master' into feature/interpreter/entity_selection
* Merge branch 'master' into feature/interpreter/entity_selection
* Merge branch 'master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge branch 'feature/interpreter/entity_selection' into feature/interpreter/refactoring_entity
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/refactoring_entity
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Contributors: Shota Minami, Tatsuya Yamasaki

3.2.0 (2024-07-18)
------------------
* Merge remote-tracking branch 'origin/master' into fix/spawn_position_of_map_pose
* Contributors: Masaya Kataoka

3.1.0 (2024-07-16)
------------------
* Merge branch 'master' into autoware_lanelet2_extension
* Merge branch 'master' into autoware_lanelet2_extension
* Contributors: Tatsuya Yamasaki

3.0.3 (2024-07-12)
------------------
* Merge branch 'master' into test/synchronized-action-kashiwanoha-map
* Contributors: Masaya Kataoka

3.0.2 (2024-07-11)
------------------

3.0.1 (2024-07-10)
------------------
* Merge branch 'master' into feature/docker_tag
* Contributors: Tatsuya Yamasaki

3.0.0 (2024-07-10)
------------------
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1053-implement-pose-utils' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge master->ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Tatsuya Yamasaki

2.6.0 (2024-07-08)
------------------
* Bump version of scenario_simulator_v2 from version 2.4.2 to version 2.5.0
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Contributors: Masaya Kataoka, Release Bot

2.4.2 (2024-07-08)
------------------

2.4.1 (2024-07-05)
------------------

2.4.0 (2024-07-01)
------------------
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Contributors: Kotaro Yoshimoto

2.3.0 (2024-06-28)
------------------
* Merge branch 'master' into feature/synchronized_action
* Merge commit 'c50d79fce98242d76671360029b97c166412e76f' into feature/synchronized_action
* Merge remote-tracking branch 'origin/master' into feature/synchronized_action
* Merge commit 'bf6a962e14e3e85627fca226574120cdba30080e' into feature/synchronized_action
* Merge commit 'bd366bce147e65d5991b62db333cf35153dd96fb' into feature/synchronized_action
* Merge commit 'b03fd92759845935be79f7ac32366848c78a2a66' into feature/synchronized_action
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronized_action
* Merge commit '45d42a79d92c370387749ad16c10665deb42e02c' into feature/synchronized_action
* Merge branch 'master' into feature/synchronized_action
* Merge commit '1ceb05c7206e163eb8214ceb68f5e35e7880d7a4' into feature/synchronized_action
* Merge commit 'f74901b45bbec4b3feb288c4ad86491de642f5ca' into feature/synchronized_action
* Merge commit '8a9b141aaf6cf5a58f537781a47f66e4c305cea3' into feature/synchronized_action
* Merge branch 'master' into feature/synchronized_action
* Merge commit '27266909414686613cea4f9aa17162d33ecf4668' into feature/synchronized_action
* Merge commit 'ada77d59ffd6545105e40e88e4ad50050062a3d6' into feature/synchronized_action
* Merge commit '253fa785573217ad3a6bde882724a9e35a0c99ed' into feature/synchronized_action
* Contributors: Masaya Kataoka, hakuturu583, koki suzuki

2.2.2 (2024-06-28)
------------------

2.2.1 (2024-06-27)
------------------
* Merge remote-tracking branch 'origin/master' into fix/issue1276-re
* Contributors: Masaya Kataoka

2.2.0 (2024-06-24)
------------------
* Merge branch 'master' into feature/clear_route_api
* Merge remote-tracking branch 'origin/master' into feature/clear_route_api
* Merge branch 'master' into feature/clear_route_api
* Merge branch 'master' into feature/clear_route_api
* Contributors: Masaya Kataoka, Taiga

2.1.11 (2024-06-24)
-------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/miscellaneous
* resolve merge confilct
* resolve merge
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/miscellaneous
* Contributors: robomic

2.1.10 (2024-06-24)
-------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/misc_object_entity
* Contributors: robomic

2.1.9 (2024-06-24)
------------------

2.1.8 (2024-06-20)
------------------
* Merge branch 'master' into feature/simple_sensor_simulator_unit_test
* Merge branch 'master' into feature/simple_sensor_simulator_unit_test
* Contributors: Kotaro Yoshimoto, SzymonParapura

2.1.7 (2024-06-19)
------------------
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Contributors: Masaya Kataoka, Mateusz Palczuk

2.1.6 (2024-06-18)
------------------

2.1.5 (2024-06-18)
------------------

2.1.4 (2024-06-14)
------------------
* Merge pull request `#1281 <https://github.com/tier4/scenario_simulator_v2/issues/1281>`_ from tier4/fix/remove_quaternion_operation
  Remove quaternion_operation
* Merge branch 'master' into fix/remove_quaternion_operation
* Update common/math/geometry/include/geometry/quaternion/get_rotation.hpp
* fix
* Merge branch 'master' into fix/remove_quaternion_operation
* Remove quaternion_operation
* Contributors: Masaya Kataoka, Taiga Takano

2.1.3 (2024-06-14)
------------------
* Merge branch 'master' into fix/issue1276
* Contributors: Masaya Kataoka

2.1.2 (2024-06-13)
------------------
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Contributors: Tatsuya Yamasaki, yamacir-kit

2.1.1 (2024-06-11)
------------------
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/reorder
* Contributors: Kotaro Yoshimoto, hakuturu583

2.1.0 (2024-06-11)
------------------
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Contributors: DMoszynski, Tatsuya Yamasaki

2.0.5 (2024-06-11)
------------------
* merge / resolve confict
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Contributors: robomic

2.0.4 (2024-06-10)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* Contributors: robomic

2.0.3 (2024-06-10)
------------------
* Merge pull request `#1263 <https://github.com/tier4/scenario_simulator_v2/issues/1263>`_ from tier4/fix/remove_linear_algebra
  Fix/remove linear algebra
* Update common/math/geometry/include/geometry/quaternion/make_quaternion.hpp
* Update common/math/geometry/include/geometry/vector3/vector3.hpp
* update test
* Merge branch 'master' into fix/remove_linear_algebra
* reformat
* fix format
* fix lint
* update
* Update internal_angle.hpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update inner_product.hpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* remove comment
* remove linear_algebra
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

2.0.2 (2024-06-03)
------------------

2.0.1 (2024-05-30)
------------------
* Merge branch 'master' into refactor/openscenario_validator
* Merge branch 'master' into refactor/openscenario_validator
* Contributors: Kotaro Yoshimoto

2.0.0 (2024-05-27)
------------------
* Merge pull request `#1233 <https://github.com/tier4/scenario_simulator_v2/issues/1233>`_ from tier4/ref/RJD-1054-implement-distance-utils
  ref(traffic_simulator): implement separate class for distance calculations, adapt make positions in SimulatorCore
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(cpp_mock, simulator_core, pose): improve names
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator, geometry): rename get2DPolygon to toPolygon2D, avoid abbreviation to bbox
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator): tidy up distance utils, move get2DPolygon to bbox
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Tatsuya Yamasaki

1.18.0 (2024-05-24)
-------------------
* Merge branch 'master' into feature/traffic-source
* Merge branch 'master' into feature/traffic-source
* Merge remote-tracking branch 'origin/master' into feature/traffic-source
* Merge branch 'master' into feature/traffic-source
* Contributors: Mateusz Palczuk, Tatsuya Yamasaki

1.17.2 (2024-05-22)
-------------------

1.17.1 (2024-05-21)
-------------------
* Merge pull request `#1255 <https://github.com/tier4/scenario_simulator_v2/issues/1255>`_ from tier4/fix/visualization
  Fix/visualization
* fix operator*
* Contributors: Kotaro Yoshimoto, hakuturu583

1.17.0 (2024-05-16)
-------------------
* Merge remote-tracking branch 'origin/master' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Merge remote-tracking branch 'origin/master' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Merge remote-tracking branch 'origin/master' into feature/openscenario_validator
* Merge remote-tracking branch 'origin/feature/openscenario_validator' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Merge branch 'master' into feature/openscenario_validator
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

1.16.4 (2024-05-15)
-------------------
* Merge branch 'master' into feature/remove_entity_type_list
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_entity_type_list
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_entity_type_list
* Merge remote-tracking branch 'origin/feature/remove_entity_type_list' into feature/remove_entity_type_list
* Merge branch 'master' into feature/remove_entity_type_list
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, hakuturu583

1.16.3 (2024-05-13)
-------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/contributing_md
* Contributors: hakuturu583

1.16.2 (2024-05-10)
-------------------

1.16.1 (2024-05-10)
-------------------
* Merge branch 'master' into doc/support_awesome-pages
* Contributors: Taiga

1.16.0 (2024-05-09)
-------------------
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge remote-tracking branch 'origin-ssh/master' into feature/respawn-entity
* Contributors: DMoszynski, Dawid Moszyński, Paweł Lech, Tatsuya Yamasaki

1.15.7 (2024-05-09)
-------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/speed_up_set_other_status
* Merge remote-tracking branch 'origin/master' into feature/speed_up_set_other_status
* Contributors: hakuturu583

1.15.6 (2024-05-07)
-------------------
* Merge branch 'master' into feature/publish_scenario_frame
* Merge remote-tracking branch 'origin/feature/publish_scenario_frame' into feature/publish_scenario_frame
* Merge branch 'master' into feature/publish_scenario_frame
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, hakuturu583

1.15.5 (2024-05-07)
-------------------

1.15.4 (2024-05-01)
-------------------

1.15.3 (2024-04-25)
-------------------
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* Contributors: Piotr Zyskowski

1.15.2 (2024-04-23)
-------------------
* Merge branch 'master' into feature/update_default_architecture_type
* Contributors: Masaya Kataoka

1.15.1 (2024-04-18)
-------------------
* Merge branch 'master' into fix/occluded-object-in-grid
* Bump version of scenario_simulator_v2 from version 1.14.1 to version 1.15.0
* Merge branch 'master' into fix/occluded-object-in-grid
* Merge branch 'master' into refactor/drop_workflow
* Merge remote-tracking branch 'origin/master' into refactor/drop_workflow
  # Conflicts:
  #	test_runner/scenario_test_runner/config/workflow_example.yaml
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Contributors: Kotaro Yoshimoto, hakuturu583, ぐるぐる

1.14.1 (2024-04-12)
-------------------

1.14.0 (2024-04-12)
-------------------
* Merge pull request `#1229 <https://github.com/tier4/scenario_simulator_v2/issues/1229>`_ from tier4/feature/follow_trajectory_action_in_do_nothing_plugin
  add follow trajectory action in do_nothing_plugin
* add follow trajectory action in do_nothing_plugin
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

1.13.0 (2024-04-11)
-------------------
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Contributors: Kotaro Yoshimoto, yamacir-kit

1.12.0 (2024-04-10)
-------------------
* Merge branch 'master' into feature/user-defined-controller
* Merge branch 'master' into feature/user-defined-controller
* Merge remote-tracking branch 'origin/master' into feature/user-defined-controller
* Contributors: Tatsuya Yamasaki, yamacir-kit

1.11.3 (2024-04-09)
-------------------
* Merge branch 'master' into refactor/basic_types
* Merge branch 'master' into refactor/basic_types
* Merge branch 'master' into refactor/basic_types
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

1.11.2 (2024-04-08)
-------------------

1.11.1 (2024-04-05)
-------------------

1.11.0 (2024-04-02)
-------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Merge remote-tracking branch 'origin/master' into feature/arm_support
* Merge remote-tracking branch 'upstream/master' into feature/arm_support
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Contributors: Masaya Kataoka, f0reachARR

1.10.0 (2024-03-28)
-------------------
* Merge branch 'master' into feature/simple_sensor_simulator/custom_noise
* Merge branch 'master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Contributors: Tatsuya Yamasaki, yamacir-kit

1.9.1 (2024-03-28)
------------------

1.9.0 (2024-03-27)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge remote-tracking branch 'origin/master' into random-test-runner-docs-update
* Contributors: Masaya Kataoka, Paweł Lech, Piotr Zyskowski

1.8.0 (2024-03-25)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_behavior_parameter_in_object_controller
* Contributors: Masaya Kataoka

1.7.1 (2024-03-21)
------------------

1.7.0 (2024-03-21)
------------------

1.6.1 (2024-03-19)
------------------

1.6.0 (2024-03-14)
------------------

1.5.1 (2024-03-13)
------------------

1.5.0 (2024-03-12)
------------------
* Merge pull request `#1209 <https://github.com/tier4/scenario_simulator_v2/issues/1209>`_ from tier4/feature/ego_slope
  Consider road slope in distance measurement and entity poses
* chore: enable flag defaultly
* Update common/math/geometry/src/spline/hermite_curve.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* chore: apply formatter
* feat: add fill_pitch option to CatmullRomSpline::getPose
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* chore(geometry): add some tests for LineSegment::getPose
* refactor: rename fit_orientation_to_lanelet to fill_pitch in HermiteCurve::getPose
* refactor: add fill_pitch option to LineSegment::getPose
* refactor: add fit_orientation_to_lanelet option to HermiteCurve::getPose
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge branch 'master' into feature/ego_slope
* fix test case
* update slop calculation logic
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka

1.4.2 (2024-03-01)
------------------

1.4.1 (2024-02-29)
------------------

1.4.0 (2024-02-26)
------------------
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Contributors: Dawid Moszyński

1.3.1 (2024-02-26)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Contributors: Masaya Kataoka

1.3.0 (2024-02-26)
------------------
* Merge remote-tracking branch 'origin/master' into feature/mrm_behavior/pull_over
* Merge remote-tracking branch 'origin/master' into feature/mrm_behavior/pull_over
* Merge remote-tracking branch 'origin/master' into feature/mrm_behavior/pull_over
* Merge remote-tracking branch 'origin/master' into feature/mrm_behavior/pull_over
* Merge remote-tracking branch 'origin/master' into feature/mrm_behavior/pull_over
  # Conflicts:
  #	external/concealer/src/field_operator_application_for_autoware_universe.cpp
* Contributors: Kotaro Yoshimoto

1.2.0 (2024-02-22)
------------------
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Merge branch 'master' into feature/default_matching_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Contributors: Masaya Kataoka

1.1.0 (2024-02-22)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

1.0.3 (2024-02-21)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/release_description
* Contributors: Masaya Kataoka

1.0.2 (2024-02-21)
------------------
* Merge remote-tracking branch 'origin/master' into doc/lane_pose_calculation
* Merge remote-tracking branch 'origin/master' into doc/lane_pose_calculation
* Bump version of scenario_simulator_v2 from version 0.8.11 to version 0.8.12
* Bump version of scenario_simulator_v2 from version 0.8.10 to version 0.8.11
* Bump version of scenario_simulator_v2 from version 0.8.9 to version 0.8.10
* Bump version of scenario_simulator_v2 from version 0.8.8 to version 0.8.9
* Bump version of scenario_simulator_v2 from version 0.8.7 to version 0.8.8
* Bump version of scenario_simulator_v2 from version 0.8.6 to version 0.8.7
* Merge branch 'master' of https://github.com/merge-queue-testing/scenario_simulator_v2 into fix/release_text
* Bump version of scenario_simulator_v2 from version 0.8.5 to version 0.8.6
* Merge branch 'master' of https://github.com/merge-queue-testing/scenario_simulator_v2 into fix/release_text
* Bump version of scenario_simulator_v2 from version 0.8.4 to version 0.8.5
* Bump version of scenario_simulator_v2 from version 0.8.3 to version 0.8.4
* Bump version of scenario_simulator_v2 from version 0.8.2 to version 0.8.3
* Bump version of scenario_simulator_v2 from version 0.8.1 to version 0.8.2
* Merge branch 'master' of https://github.com/merge-queue-testing/scenario_simulator_v2 into feature/restore_barnch
* Bump version of scenario_simulator_v2 from version 0.8.0 to version 0.8.1
* Merge pull request `#1 <https://github.com/tier4/scenario_simulator_v2/issues/1>`_ from merge-queue-testing/feature/new_release
  Feature/new release
* Merge pull request `#10 <https://github.com/tier4/scenario_simulator_v2/issues/10>`_ from hakuturu583/test/release
  update CHANGELOG
* update CHANGELOG
* Contributors: Masaya Kataoka, Release Bot

1.0.1 (2024-02-15)
------------------

1.0.0 (2024-02-14)
------------------
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into feature/real-time-factor-control
* Merge branch 'tier4:master' into random-test-runner-docs-update
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Contributors: Paweł Lech, pawellech1, yamacir-kit

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
