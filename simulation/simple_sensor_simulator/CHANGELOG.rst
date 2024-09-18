^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_sensor_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'master' into RJD-1278/geometry-update
* Merge branch 'master' into RJD-1278/geometry-update
* Merge branch 'master' into RJD-1278/geometry-update
* Merge branch 'master' into RJD-1278/geometry-update
* Contributors: Masaya Kataoka, Michał Ciasnocha

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
* Revert "feat(params): set use_sim_time default as True"
  This reverts commit da85edf4956083563715daa3d60f0da1f94a423d.
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* feat(params): set use_sim_time default as True
* Merge remote-tracking branch 'origin/master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into doc/RJD-1273-add-realtime-factor-doc
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto

4.1.0 (2024-09-03)
------------------
* Merge branch 'master' into RJD-1278/fix-line-segment
* Merge branch 'master' into RJD-1278/fix-line-segment
* Merge branch 'master' into RJD-1278/fix-line-segment
* Merge branch 'master' into RJD-1278/fix-1344-getIntersection2DSValue
* Merge branch 'master' into RJD-1278/fix-1343-isIntersect2D
* Contributors: Michał Ciasnocha

4.0.4 (2024-09-02)
------------------
* Merge pull request `#1301 <https://github.com/tier4/scenario_simulator_v2/issues/1301>`_ from tier4/feature/simple_sensor_simulator_unit_tests_lidar
  Test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Removed dummy class
  - Updated unit tests
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Merge branch 'master' into feature/simple_sensor_simulator_unit_tests_lidar
* Test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Added missed header file
* Test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Added unit tests to LidarSensor
  - Addede unit tests to Primitive
  - Refactored Raycaster unit tests
* Contributors: Masaya Kataoka, SzymonParapura

4.0.3 (2024-08-29)
------------------
* Merge pull request `#1358 <https://github.com/tier4/scenario_simulator_v2/issues/1358>`_ from tier4/RJD-1056-remove-npc-logic-started
  Remove unused data members: npc_logic_started
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
* ref(ego_entity_simulation): slight improve - add const, rename current_time
* fix(ego_entity): fix autoware update when not npc_logic_started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'origin/ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Contributors: DMoszynski, Dawid Moszynski, Masaya Kataoka, Mateusz Palczuk

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
* Merge pull request `#1320 <https://github.com/tier4/scenario_simulator_v2/issues/1320>`_ from tier4/ref/RJD-1053-set-update-canonicalized-entity-status
  ref(behavior_tree, traffic_simulator): move responsibility for canonicalization to traffic_simulator, simplify
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
* doc(developer_guide, traffic_simulator): update doc after review changes, add code notes
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
* doc(developer_guide, pose utils): adopt lane_pose_calculation doc to canonicalization laneletpose in CanonicalizedEntityStatus
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* ref(traffic_simulator): move toCanonicalizeLaneletPose to CanonicalizedEntityStatus::set, little tidy up
* ref(traffic_simulator): remove operator= for CanonicalizedEntityStatus, use set and assertions
* Merge remote-tracking branch 'origin/ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Mateusz Palczuk, Tatsuya Yamasaki

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
* Merge branch 'master' into doc/longitudinal-control
* Merge pull request `#1321 <https://github.com/tier4/scenario_simulator_v2/issues/1321>`_ from tier4/feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge commit 'c1cab6eb1ece2df58982f50a78fef5a5ecaa7234' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* feat: Enhance IMU sensor configuration and initialization
  - Added frame_id to ImuSensorConfiguration
  - Separated noise standard deviations for orientation, twist, and acceleration
  - Updated ImuSensorBase and ImuSensor classes for new noise distributions
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Fix an issue with an invalid namespace imu_link
* Fix an issue with an invalid namespace imu_link
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* feat(simple_sensor_simulator, imu): add gravity vector, tidy up
* feat(simulator_core, api, zmq): add attachImuSensor, add update imu sensors
* feat(simple_sensor_simulator): add imu_sensor
* Contributors: Dawid Moszynski, Koki Suzuki, Kotaro Yoshimoto, Masaya Kataoka, SzymonParapura, koki suzuki

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
* Merge pull request `#1266 <https://github.com/tier4/scenario_simulator_v2/issues/1266>`_ from tier4/ref/RJD-1053-implement-pose-utils
  ref(traffic_simulator): extend utils/pose - use it globally, improve canonization process
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* feat(pose utils): apply requested changes
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
* ref(traffic_simulator): global improvements, comments, revert unnecessary changes
* feat(traffic_simulator): use consider_pose_by_road_slope as static variable in CanonicaliedLaneletPose
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(simulator_core, ego_entity_simulation): improve strings
* ref(simulator_core,sss,pose): revert unintended changes
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(simple_sensor_simulator): reduce the number of changes in EgoEntitySimulation
* ref(simple_sensor_simulator): remove fillLaneletDataAndSnapZToLanelet
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
* remove simple sensor simulator tets
* vertex, toPoints
* simple_sensr_simulation unit tests
* Contributors: robomic

2.1.10 (2024-06-24)
-------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/misc_object_entity
* Contributors: robomic

2.1.9 (2024-06-24)
------------------

2.1.8 (2024-06-20)
------------------
* Merge pull request `#1291 <https://github.com/tier4/scenario_simulator_v2/issues/1291>`_ from tier4/feature/simple_sensor_simulator_unit_test
  Feature/simple sensor simulator unit test
* Moved ament_cmake_gtest to package.xml
* Merge branch 'master' into feature/simple_sensor_simulator_unit_test
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Fixed an issue with missed header file
* Merge branch 'master' into feature/simple_sensor_simulator_unit_test
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Fixed an issue with missing package
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Corrected directory structure
  - Fixed an issue with a mistake
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Replaced constructors with SetUp: RaycasterTest and GridTraversalTest
  - Refactored Box, GridTraversal and Vertex unit tests
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Fixed an issue with Raycaster tests - pointcloud size checking
  - Updated test descriptions
  - Added 'const' to specific variables
  - Ensured a new line is added at the end of each file
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Added CMakeLists.txt
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Added unit tests to GridTraversal
* test: [RJD-937] to Implement Unit tests on simple_sensor_simulator
  - Added unit tests to Box and Vertex classes
  - Added unit tests to Raycaster class
  - Added set of macros used in unit tests
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
* Merge branch 'master' into fix/remove_quaternion_operation
* fix package xml
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
* Merge pull request `#1279 <https://github.com/tier4/scenario_simulator_v2/issues/1279>`_ from tier4/fix/reorder
  fix -wreorder warning
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/reorder
* fix -wreorder warning
* Contributors: Kotaro Yoshimoto, hakuturu583

2.1.0 (2024-06-11)
------------------
* Merge pull request `#1226 <https://github.com/tier4/scenario_simulator_v2/issues/1226>`_ from tier4/fix/RJD-955-fix-followtrajectoryaction-nan-time
  fix(follow_trajectory_action): adapt to work with considering slopes
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* fix(ego_entity_simulation): fix assignment of world_relative_position\_ when not npc_logic_started
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* fix(ego_entity_simulation): fix world_relative_position\_ for update()
* Revert "feat(ego_entity_simulation): develop VehicleModelState"
  This reverts commit 1f72837309a4055aa750b1ec2c5e31b50c6a65b6.
* feat(ego_entity_simulation): develop VehicleModelState
* ref(ego_entity_simulation): use world_relative_position\_ in getCurrentPose and calculateEgoPitch
* ref(ego_entity_simulation): use world_relative_position, use only Oz
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* ref(ego_entity_simulation): apply clang reforamt
* ref(ego_entity_simulation, sim_model_interface): use world_relative_position_z\_ to store Oz position
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* fix(ego_entity_simulation, sim_model_interface): add overwrite position in Oz axis
* fix(toLaneletPose): fix matching_distance in EgoEntity, EgoEntitySimulation and BehaviorTree
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Tatsuya Yamasaki

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
* Merge branch 'master' into fix/remove_linear_algebra
* Contributors: Taiga

2.0.2 (2024-06-03)
------------------

2.0.1 (2024-05-30)
------------------
* Merge branch 'master' into refactor/openscenario_validator
* Merge branch 'master' into refactor/openscenario_validator
* Contributors: Kotaro Yoshimoto

2.0.0 (2024-05-27)
------------------
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Contributors: DMoszynski, Dawid Moszynski, Masaya Kataoka, Tatsuya Yamasaki

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
* Merge pull request `#1198 <https://github.com/tier4/scenario_simulator_v2/issues/1198>`_ from tier4/feature/respawn-entity
  Feature/respawn entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* fix(sss): missing adaptation change - get_parameter
* ref(sss, concealer): apply requested PR changes - style
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* ref(traffic_simulator, respawn): apply requested PR changes
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge remote-tracking branch 'origin-ssh/master' into feature/respawn-entity
* Reducing the execution time of EgoEntitySimulation::makeSimulationModel
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Paweł Lech, Tatsuya Yamasaki

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
* Contributors: Piotr Zyskowski

1.15.2 (2024-04-23)
-------------------
* Merge branch 'master' into feature/update_default_architecture_type
* Contributors: Masaya Kataoka

1.15.1 (2024-04-18)
-------------------
* Merge pull request `#1227 <https://github.com/tier4/scenario_simulator_v2/issues/1227>`_ from tier4/fix/occluded-object-in-grid
  Exclude LiDAR occluded object on OccupancyGrid
* Merge branch 'master' into fix/occluded-object-in-grid
* Bump version of scenario_simulator_v2 from version 1.14.1 to version 1.15.0
* Merge branch 'master' into fix/occluded-object-in-grid
* Merge branch 'master' into refactor/drop_workflow
* Merge remote-tracking branch 'origin/master' into refactor/drop_workflow
  # Conflicts:
  #	test_runner/scenario_test_runner/config/workflow_example.yaml
* Fix spell
* Exclude LiDAR occluded object from OccupancyGrid in simple_sensor_simulator
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR, hakuturu583, ぐるぐる

1.14.1 (2024-04-12)
-------------------

1.14.0 (2024-04-12)
-------------------

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
* Merge pull request `#1173 <https://github.com/tier4/scenario_simulator_v2/issues/1173>`_ from tier4/feature/arm_support
  Feature/arm_build_test
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Merge remote-tracking branch 'origin/master' into feature/arm_support
* Merge remote-tracking branch 'upstream/master' into feature/arm_support
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* fix compile error
* remove IntersectContext
* remove unused function
* remove unused line
* rename to new struct name
  https://github.com/embree/embree/blob/be0accfd0b246e2b03355b8ee7710a22c1b49240/README.md?plain=1#L1360-L1361
* remove some warnings
* change header
* add arm support branch
* Contributors: Masaya Kataoka, Ubuntu, f0reachARR

1.10.0 (2024-03-28)
-------------------
* Merge pull request `#1200 <https://github.com/tier4/scenario_simulator_v2/issues/1200>`_ from tier4/feature/simple_sensor_simulator/custom_noise
  Feature/simple sensor simulator/custom noise
* Merge branch 'master' into feature/simple_sensor_simulator/custom_noise
* Merge branch 'master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/custom_noise
* Remove the upper limit `300.0` of the detection sensor range
* Replace the word `TheEntity` with `EgoEntity`
* Rename function argument `detected_object` to `detected_objects`
* Update `DefaultNoiseApplicator` to hold reference to Ego entity's status
* Simplify complex and inefficient function `filterObjectsBySensorRange`
* Add new struct `CustomNoiseApplicator`
* Rename variables to appropriate for the current class content
* Move the process of applying noise to a new structure `DefaultNoise`
* Add type U for ground truth to the template parameter of DetectionSensor
* Split ROS message type object construction into some free functions
* Move the DetectedObject construction to a new free function
* Lipsticks
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki, yamacir-kit

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
* doc: use 3 slashes to comment-out before doxygen command
* fix: use fill_pitch option in EgoEntitySimulation::fillLaneletDataAndSnapZToLanelet
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* doc:
* chore: apply formatter
* fix(build)
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* chore: format
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
* chore: fix conflict resolving miss
* doc: add note for origin orientation of grid map
* fix: build error
* chore: format
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* refactor: use consider_pose_by_road_slope instead of consider_lanelet_pose
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* update slop calculation logic
* enable toggle setting
* add consider_lanelet_slope member value
* remove warnings
* add considerLaneletSlope() function
* consider lanalet slope
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

1.4.2 (2024-03-01)
------------------

1.4.1 (2024-02-29)
------------------

1.4.0 (2024-02-26)
------------------
* Merge pull request `#1163 <https://github.com/tier4/scenario_simulator_v2/issues/1163>`_ from tier4/fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
  fix(follow_trajectory_action): fix cooperation with Autoware, fix speed limits
* fix(ego_entity_simulation): fix after merge
* ref(ego_entity_simulation): apply clang reformat
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge branch 'master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* ref(sss): remove unused include
* ref(sss,simulation_interface, ego_entity): apply ament_clang reformat
* feat(ego_entity): provide FollowTrajectoryAction execution in EgoEntity, slight ref FollowTrajectoryAction
* feat(sss): add option to overwrite Ego status
* feat(zmq,sss): remove FollowPolylineTrajectoryRequest
* Revert "feat(sss): allow target_speed and max_speed to be set in EgoEntitySimulation"
  This reverts commit f8f70d2ae1b4c7c4b91ba0af8938bcadcfb71545.
* ref(ego_entity_simulator, proto): review changes
* ref(ego_entity_simulation): calc target_speed only for npc_logic_started
* fx(ego_entity_simulation): fix commit 'fix ego target_speed'..
* fix(ego_entity_simulation): fix ego target_speed without lanelet valid pose
* ref(simulation): apply clang reformat
* feat(sss): allow target_speed and max_speed to be set in EgoEntitySimulation
* Contributors: Dawid Moszyński, Tatsuya Yamasaki

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
* Contributors: Kotaro Yoshimoto

1.2.0 (2024-02-22)
------------------
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Merge branch 'master' into feature/default_matching_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Contributors: Masaya Kataoka

1.1.0 (2024-02-22)
------------------
* Merge pull request `#1182 <https://github.com/tier4/scenario_simulator_v2/issues/1182>`_ from tier4/feature/slope_vehicle_model
  Consider road slope in ego vehicle simulation
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/slope_vehicle_model
* Update simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* chore: format
* fix(EgoEntitySimulation)
* refactor(EgoEntitySimulation): convert lane pose matching processing to getMatchedLaneletPoseFromEntityStatus function
* fix: pass consider_acceleration_by_road_slope to inside of EgoEntitySimulator
* doc: add notification to duplicated lane matching algorithm
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* chore: format
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
  # Conflicts:
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* refactor(ego_entity_simulation): rename flag name for considering slope in ego entity simulation
* feat(ego_entity_simulation): add flog for considering slope in ego entity simulation
* feat(ego_entity_simulation): consider slope in ego entity simulation
* doc: add memos to code
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
* Merge remote-tracking branch 'test/master' into feature/new_release
* Merge pull request `#10 <https://github.com/tier4/scenario_simulator_v2/issues/10>`_ from hakuturu583/test/release
  update CHANGELOG
* update CHANGELOG
* Contributors: Masaya Kataoka, Release Bot

1.0.1 (2024-02-15)
------------------

1.0.0 (2024-02-14)
------------------
* Merge pull request `#1184 <https://github.com/tier4/scenario_simulator_v2/issues/1184>`_ from tier4/feature/consider_tread_in_ego_entity
  Feature/consider tread in ego entity
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/new_release_flow
* Merge branch 'master' into fix/autoware-shutdown
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/consider_tread_in_ego_entity
* Merge pull request `#1150 <https://github.com/tier4/scenario_simulator_v2/issues/1150>`_ from tier4/feature/real-time-factor-control
  Feature/real time factor control
* add setTwist/setAcceleration function
* extend matching length
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into feature/real-time-factor-control
* Merge branch 'tier4:master' into random-test-runner-docs-update
* Setting concealer use_sim_time manually instead of using global arguments.
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* UpdateStepTime request for updating simple sensor simulation step_time
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Paweł Lech, Tatsuya Yamasaki, pawellech1, yamacir-kit

0.9.0 (2023-12-21)
------------------
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* Merge pull request `#1144 <https://github.com/tier4/scenario_simulator_v2/issues/1144>`_ from tier4/feature/update_sim_model
* doc: change comment style with note commands
* doc: fix note command place
* chore(simple_sensor_simulator): fix indent
* chore(simple_sensor_simulator): use @note command
* chore(simple_sensor_simulator): delete acceleration_map.csv
* feat(simple_sensor_simulator): import sim_model_delay_steer_map_acc_geared
* feat(simple_sensor_simulator): import steer dead band feature
* refactor: reflect the reviews
* refactor(simple_sensor_simulator): delete name of unused argument to suppress warnings
* change default port to 5555
* change default port to 8080
* change port
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge pull request `#1069 <https://github.com/tier4/scenario_simulator_v2/issues/1069>`_ from tier4/feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into refactor/lanelet-id
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge pull request `#1087 <https://github.com/tier4/scenario_simulator_v2/issues/1087>`_ from tier4/feature/drop_galactic_support
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/drop_galactic_support
* Merge pull request `#1027 <https://github.com/tier4/scenario_simulator_v2/issues/1027>`_ from tier4/feature/new_traffic_light
* refactor(traffic_simulator): change to a comparison method that is resistant to version changes
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* refactor(simple_sensor_simulator): add explicit keyword to constructor
* refactor(simple_sensor_simulator): TrafficLightDetectorEmulator => PseudoTrafficLightDetector
* refactor(simple_sensor_simulator): TrafficLightDetectorEmulator => PseudoTrafficLightDetector
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* remove workbound for galactic
* refactor: TrafficLightDetectorEmulator => PseudoTrafficLightDetector
* chore: change architecture_type to awf/universe/20230906
* fix: build errors
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Update `update` to use `updateStatus` instead of `setStatus`
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Update the Ego entity's FollowTrajectoryAction to take into account the vehicle model type
* Update yaw calculation during `FollowTrajectoryAction` for Ego entities
* Rename argument `time` to `current_scenario_time`
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/ref/RJD-553_restore_repeated_update_entity_status' into pzyskowski/660/ss2-awsim-connection
* Update `EgoEntitySimulation::update` to follow trajectory if is given
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* refactor: apply formatter
* fix: build errors
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Update `followPolylineTrajectory` to store the given trajectory to ego
* Simplify member function `ScenarioSimulator::isEgo`
* Simplify member function `ScenarioSimulator::isEntityExists`
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* refactor: update architecture_type format
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* fix(simple_sensor_simulator): update topic name for conventional traffic light
* refactor(simple_sensor_simulator): delete debug code
* refactor(simple_sensor_simulator): move include
* feat: add new architecture_type awf/universe/2023.08
* refactor: delete unused lines/files
* refactor(simple_sensor_simulator): delete TrafficLightsDetectorBase
* chore: apply formatter
* feat(simple_sensor_simulator): implement ScenarioSimulator::attachTrafficLightDetectorEmulator
* refactor(simple_sensor_simulator): use TrafficLightPublisher in simple_sensor_simulator
* refactor(simple_sensor_simulator): use UpdateTrafficLightsRequest as a type for storing
* feat(simple_sensor_simulator): add base class for TrafficLightsDetector
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Revert "WIP: get and sets for vehicle status"
* WIP: get and sets for vehicle status
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Mateusz Palczuk, Michał Kiełczykowski, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge pull request `#1058 <https://github.com/tier4/scenario_simulator_v2/issues/1058>`_ from tier4/ref/RJD-553_restore_repeated_update_entity_status
* fix(sss): fix missing initialization
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1074 <https://github.com/tier4/scenario_simulator_v2/issues/1074>`_ from tier4/fix/clock
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1024 <https://github.com/tier4/scenario_simulator_v2/issues/1024>`_ from tier4/feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* refactor: detection_sensor.hpp sensor_simulation.hpp
* fix: make sure generated uuid is valid
* refactor: detection_sensor.hpp
* chore: apply linter
* feat: add implementation for detectedObjectGroundTruthPublishingDelay
* refactor: change property name from isEnableDetectedObjectGroundTruthDelay to detectedObjectGroundTruthPublishingDelay
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1056 <https://github.com/tier4/scenario_simulator_v2/issues/1056>`_ from tier4/feature/interpreter/sensor-detection-range
* Update the `simple_sensor_simulator` to pass the scenario time to the ego entity
* Rename API `UpdateFrameRequest::current_time` to `current_simulation_time`
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* fix(sss): fix mistake - spawned entity type
* ref(clang): apply clang reformat
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* ref(single_sensor_simulator): apply changes requested in review
* Merge pull request `#1061 <https://github.com/tier4/scenario_simulator_v2/issues/1061>`_ from tier4/feature/traffic_simulator/follow-trajectory-action-2
* merge lidar publishing delay
* reformat
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* ref(traffic_simulator,sss): apply clang_reformat
* Merge pull request `#1018 <https://github.com/tier4/scenario_simulator_v2/issues/1018>`_ from tier4/fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* revert change for occupancygrid
* revert change for occupancygrid
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* refactor distance between pose
* Update simulation/simple_sensor_simulator/src/sensor_simulation/detection_sensor/detection_sensor.cpp
* Update simulation/simple_sensor_simulator/src/sensor_simulation/detection_sensor/detection_sensor.cpp
* Update simulation/simple_sensor_simulator/include/simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp
* Update simulation/simple_sensor_simulator/include/simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge pull request `#1046 <https://github.com/tier4/scenario_simulator_v2/issues/1046>`_ from tier4/fix/RJD-554_error_run_scenario_in_row
* fix(zeromq): ensure single update ego, optimize UpdateEntityStatus
* Update `MultiServer` to require API callbacks to return responses as return value
* Add new simulation API `FollowPolylineTrajectory(Request|Response)`
* Simplify class `MultiServer` definition
* revert lidar sensor delay's change
* apply clang-format
* fix(sss): reset ego after erase
* merge master branch
* ref(sss): add initialize entity_status\_ as empty
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* ref(sss): provide Ego update anywhere in the request, speed up isEntityExist
* ref(sss): optimize despawnEntity method
* ref(zeromq): restore repeated UpdateEntityStatus
* Merge pull request `#1054 <https://github.com/tier4/scenario_simulator_v2/issues/1054>`_ from tier4/remerge-1023
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* apply distance filter for lidar_detected_entity
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Revert "Revert "Merge pull request `#1023 <https://github.com/tier4/scenario_simulator_v2/issues/1023>`_ from tier4/feat/pointcloud_delay""
* Fix: init also entity_status\_ map
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* fix(simple_sensor_simulator): fix despawn ego, ref despawn method
* Merge remote-tracking branch 'origin/master' into feat/relative_object_position
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* feat(simple_sensor_simulator): implement ground truth delay
* refactor(simple_sensor_simulator): implement conversion from DetectedObjects to TrackedObjects
* refactor(simple_sensor_simulator): use template type
* feat(simple_sensor_simulator): add ground truth publisher to DetectionSensor
* Merge branch 'master_4284' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Kyoichi Sugahara, Lukasz Chojnacki, Masaya Kataoka, Tatsuya Yamasaki, kosuke55, kyoichi-sugahara, yamacir-kit

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* Refactor: When despawnEntity() is called, firstly check if entity isEgo() and then assign vectors with removed entity
* Introduce ifEntityExists() method. Throw error if entity not exists
* Change variable name. Add const guard
* Add const &
* Change including from  to <>
* Add explicit keyword
* galactic build fix
* added deprecated tf2 messages flag to sss cmakelists
* code cleanup
* moved vehicle simulation to simple sensor simulator
* clang format
* trafic lights moved to simple sensor simulation in unelegant manner
* moved EES to SSS
* EES initialized in SSS
* initializing hdmautils in sss
* lanelet2 map passing via zmq
* entity status zmq update
* utilizing updated entity data
* map to keep entity status in sss; zmq entity update takes one entity at a time
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* further removed updatesensorframe from zmq interface
* merged UpdateSensorFrame into UpdateFrameRequest
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#894 <https://github.com/tier4/scenario_simulator_v2/issues/894>`_ from tier4/fix/cleanup_code
* Merge branch 'master' into feature/interpreter/model3d-field
* Merge remote-tracking branch 'origin/master' into feature/interpreter/publishing-delay
* Merge branch 'master' into fix/cleanup_code
* Merge branch 'master' into feature/interpreter/environment
* Merge pull request `#981 <https://github.com/tier4/scenario_simulator_v2/issues/981>`_ from RobotecAI/ref/AJD-697_improve_port_management_zmq
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* Merge remote-tracking branch 'origin/master' into clean-dicts
* ref(sim_interface): apply ament clang reformat
* ref(zmq): add socket_port as rosparam
* Merge branch 'master' into feature/interpreter/model3d-field
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#964 <https://github.com/tier4/scenario_simulator_v2/issues/964>`_ from tier4/feature/noise_delay_object
* change variable name
* fix error
* use queue
* ament_clang_format
* fix code style
* fix variable
* replace vector with deque
* get param from interpreter
* fix conflict
* Merge branch 'master' into feature/noise_delay_object
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge pull request `#973 <https://github.com/tier4/scenario_simulator_v2/issues/973>`_ from tier4/feature/interpreter/probability-of-lost
* Update the probability range to be [0, 1) from [0, 100)
* Move member function `recognizeWithProbability` into `DetectionSensor::update`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin' into fix/getting_next_lanelet
* Merge pull request `#958 <https://github.com/tier4/scenario_simulator_v2/issues/958>`_ from tier4/feature/noise_lost_object
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* tempolary implemantation of delay recognition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* delete comment
* add function of recognizing object with probability
* change function name from applyNoise to applyPositionNoise
* added param probability of lost recognition
* Merge branch 'master' into feature/noise_lost_object
* temp
* Merge pull request `#951 <https://github.com/tier4/scenario_simulator_v2/issues/951>`_ from tier4/fix/warnings
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* remove C++ warnings
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* Merge remote-tracking branch 'origin/master' into import/universe-2437
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#937 <https://github.com/tier4/scenario_simulator_v2/issues/937>`_ from tier4/feature/interpreter/noise
* Move data member `random_engine\_` to `DetectionSensor` from `DetectionSensorBase`
* Update `DetectionSensor<>::applyNoise` to not to receive non-const reference
* Cleanup member function `DetectionSensor<...>::update`
* Move member function `applyNoise` into `DetectionSensor` from `DetectionSensorBase`
* Update `rand_engine\_` to allocate on the stack instead of the heap
* Update `position_noise_distribution` to allocate on the stack instead of the heap
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#914 <https://github.com/tier4/scenario_simulator_v2/issues/914>`_ from tier4/feature/simple_noise_simulator
* Merge branch 'master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge pull request `#922 <https://github.com/tier4/scenario_simulator_v2/issues/922>`_ from RobotecAI/ajd-618/get-rotation-matrix-optimization
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* apply format
* add member variable
* Merge pull request `#884 <https://github.com/tier4/scenario_simulator_v2/issues/884>`_ from tier4/feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Format
* Fix typo
* Add implentation commnes
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Apply clang_format
* Precompute rotation matrix
* Check if polygon is empty
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* remove omit
* update proto
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/simple_noise_simulator
* define member function as template
* change include order
* apply clang-format
* add simple noise generator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Fix typo
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Rename variable
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Add missing headers
* Fix wrong variable name
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Format
* Replace boost::optional with std::optional
* Clip occupied area by grid square
* Fix filling range
* Fix infinite loop
* Use std::pair instead of original type
* Simplify for loop
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Revert "Fix bounding box shifting"
* Fix bounding box shifting
* Add missing comment
* Remove redundant utility
* Change variable name
* Fix typo
* Add overlap check
* Add internal utility
* Refactor
* Rename
* Remove debug code
* Add target file
* Change class name
* Add pixel traversal as iterator
* Update URL
* Format
* Add validation
* Rename type names
* Refactor
* Refactor
* Update Doxygen comment
* Refactor
* Format
* Complete new algorithm implementation
* Use optimized method
* Optimize filling algorithm
* Change data structure
* Implement invisible polygon calculation
* Fix division by zero bug
* Optimize line traverse
* Convert coordinate in advance
* [WIP] Impletation complete, but not works
* [WIP] Implement sweep line algorithm
* Fix wrong comment
* Improve corner calculation
* [WIP] implement sweep line
* Add missing coordinate conversion
* [WIP] implement improved algorithm
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Refactor to keep consistency
* Refactor
* Refactor
* Refactor
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Adam Krawczyk, Dawid Moszyński, Kotaro Yoshimoto, Kyoichi Sugahara, Masaya Kataoka, MasayaKataoka, Minami Shota, Shota Minami, f0reachARR, hrjp, kyoichi sugahara, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into feat/heat_beat
* Merge pull request `#913 <https://github.com/tier4/scenario_simulator_v2/issues/913>`_ from tier4/use/autoware_github_actions
* fix(typo): cooridnate => coordinate
* fix(typo): implemtation => implementation
* chore(spell-check): fix cspell error
* chore(spell-check):  fix cspell error
* Merge pull request `#888 <https://github.com/tier4/scenario_simulator_v2/issues/888>`_ from tier4/fix/shifted_bounding_box
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Copy dimension info
* Merge remote-tracking branch 'origin/master' into fix/traffic_simulator/horizon
* Merge pull request `#905 <https://github.com/tier4/scenario_simulator_v2/issues/905>`_ from tier4/fix/update-orientation-availability
* fix: orientation availability
* Merge remote-tracking branch 'origin/master' into feature/interpreter/priority
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge remote-tracking branch 'origin/master' into feature/interpreter/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into refactor/test_runner
* Merge pull request `#886 <https://github.com/tier4/scenario_simulator_v2/issues/886>`_ from RobotecAI/mkielczykowski/profile_embree
* review applied
* humble build fix
* Code cleanup: delete Todo
* Code cleanup: clang format
* Fix bounding box shifting
* Code cleanup: organise the raycaster code
* Code cleanup: delete time measurement
* Revert "WIP: Intermediate dynamic_scene optimization"
* Fix compilation
* MT patches
* WIP: Implement (para)lellization and add scenarios
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge pull request `#866 <https://github.com/tier4/scenario_simulator_v2/issues/866>`_ from tier4/fix/simple_sensor_simulator/fast_occupancy_grid
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Fix wrong comment
* Format
* Remove unnecessary declaration
* Merge branch 'feature/occupancy_grid_docs' into fix/simple_sensor_simulator/fast_occupancy_grid
* Refacor `occupancy_grid_sensor` and improve performance
* Remove redundant code and simplify
* Reformat
* WIP: Intermediate dynamic_scene optimization
* Simplify grid cell data structure to improve performance
* Fix typo
* Merge remote-tracking branch 'origin/fix/ci_error' into feature/start_npc_logic_api
* Merge branch 'master' into feature/occupancy_grid_docs
* Update doxygen comments
* Add doxygen comment
* WIP: embree optimizations
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Piotr Rybicki, Piotr Zyskowski, Shota Minami, kyabe2718, satoshi-ota, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge remote-tracking branch 'origin/master' into refactor/catalog
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge pull request `#849 <https://github.com/tier4/scenario_simulator_v2/issues/849>`_ from tier4/fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Change `Grid` to update partially
* Reorder lines
* Improve filling algorithm
* Merge pull request `#847 <https://github.com/tier4/scenario_simulator_v2/issues/847>`_ from tier4/feature/value_constraint
* Replace "Tier IV" with "TIER IV"
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into doc/6th_improvement
* Merge pull request `#835 <https://github.com/tier4/scenario_simulator_v2/issues/835>`_ from RobotecAI/fix/obstacle_detection_raycaster
* Add zero-initialization of raycast and change default mask values
* Set mask property to raycaster hit to enable intersecting with geometries in the scene
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
* add contains to the collision directory
* move to geometry math
* move line segment class to the polygon directory
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* move directory
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* use geometry_math::getMinValue/getMaxValue function
* add filterByAxis function
* use std::transform
* Merge branch 'feature/get_distance_to_lane_bound' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* standard to 17
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_lane_bound
* Merge pull request `#803 <https://github.com/tier4/scenario_simulator_v2/issues/803>`_ from tier4/feature/replace_dummy_ogm_map
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pull_over_metrics
* Merge branch 'master' into feature/change_engage_api_name
* feat!: replace dummy ogm
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge pull request `#797 <https://github.com/tier4/scenario_simulator_v2/issues/797>`_ from tier4/feature/occupancy_grid_sensor
* add white lines at EOF
* remove unused operators
* Merge pull request `#778 <https://github.com/tier4/scenario_simulator_v2/issues/778>`_ from tier4/feature/zmqpp_vendor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Fix Licence
* enable fill occupied cell
* use static_cast in position calculation
* add check in fillByRowCol
* filter row and col index
* fix length calculation
* add get2DVector get2DLength function
* add get2D Length function
* enable fill intersection cell
* add getInvisibleRay function
* remove unused function
* fix problems in fillByIntersection function
* add fillInside function
* add get rows and cols function
* add filter by row and col fuinction
* add sortAndUnique function
* fix problem in floor
* enable fill value
* fix intersection calculation algorithum
* enable generate faster
* change architecture
* remove sort and erase
* comment out invisible cell
* add debug line
* enable extract candidate
* add transform
* fill invisible cell
* implement raycastToOutside function
* add constructor
* addgetOutsideLineSegments function
* modify constructor
* add getIntersection2D function
* remove debug message
* add filterByIndex function
* add merge function
* fix compile error
* enable filter by intersection
* add filterByIntersection function
* add intersection2D function
* add getCols/Rows function
* add transformToWorld function
* fill with extra cell
* enable get cell
* fix logic
* apply reformat
* rename topic
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* call attachOccupancyGridSensor function in server
* fix compile errors
* generate occupancy grid data
* add index infomation to the cell class
* add transform function
* use const &
* add generate step
* update ego_pose_north_up logic
* fix compile errors
* fix reorder warnings
* add add primitive line
* add getMin and Max function
* add Grid class
* add intersection2D function
* add primitive
* add generator class
* add bounding box generation step
* add occupancy grid sensor class
* add nav_msgs to the depends
* attach occupancy grid sensor function to the simulator
* modify CMakeLists.txt
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, taikitanaka3, tanaka3, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#712 <https://github.com/tier4/scenario_simulator_v2/issues/712>`_ from tier4/fix/object-recognition-from-prediction-to-detection
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* add virtual destructor to DetectionSensorBase and LidarSensorBase
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#729 <https://github.com/tier4/scenario_simulator_v2/issues/729>`_ from tier4/feature/ignore_raycast_result
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* apply clanf-format
* fix enum access
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* rename data field
* enable filter by lidar detection result
* enable filter by range
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* add default case
* fix invalid return
* use switch and remove warning
* use else if
* fix compile errors in simulator
* rename to subtype
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* fix compile error
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* modify proto file
* enable filter by range
* fix ci test
* fix topic name
* use detected objects instead of predicted ones
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Remove architecture_type `awf/auto`
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#683 <https://github.com/tier4/scenario_simulator_v2/issues/683>`_ from tier4/feature/zeromq_multi_client
  Feature/zeromq multi client
* remove unused files
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Remove package `autoware_perception_msgs`
* Remove some unused dependencies
* Remove architecture_type `tier4/proposal`
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
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: MasayaKataoka, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Rename `LiDAR` to `Lidar`
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Merge pull request `#622 <https://github.com/tier4/scenario_simulator_v2/issues/622>`_ from tier4/fix-pointcloud-topic
* fix topic name of pointcloud
* Lipsticks
* Update class `SensorSimulation` to choice topic name and type based on Autoware's architecture type
* Add new virtual class `LiDARSensorBase`
* Add new virtual class `DetectionSensorBase`
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* detected object -> predicted object, and apply ament_clang_format
* Update DetectionSensor to use `autoware_auto_perception_msgs`
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
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
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
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#530 <https://github.com/tier4/scenario_simulator_v2/issues/530>`_ from RobotecAI/traffic_lights
* Clang formatting and conversions test for traffic light
* ZMQ api for traffic lights
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Contributors: Masaya Kataoka, Piotr Jaroszek, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* fix typo in rviz
* fix typo
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka

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
* did not output detection result when we found obstacle
* enable spanw/despaen misc object in simulator
* fix compile erros in sensor simulator package
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718

0.2.0 (2021-06-24)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
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
* Merge pull request `#330 <https://github.com/tier4/scenario_simulator_v2/issues/330>`_ from tier4/fix/raycast_timing
* fix compile errors
* fix sensor frame update logic
* Merge pull request `#329 <https://github.com/tier4/scenario_simulator_v2/issues/329>`_ from tier4/fix/update_sensor_frame_logic
* fix frame logic
* Merge pull request `#313 <https://github.com/tier4/scenario_simulator_v2/issues/313>`_ from tier4/fix/publish_npc_detection_result_in_map_frame
* use switch
* remove comma
* apply reformat
* enable publish detection result pose
* Merge pull request `#311 <https://github.com/tier4/scenario_simulator_v2/issues/311>`_ from tier4/feature/fix_lidar_frame_id
* apply clang-format
* change header frame id
* Merge pull request `#304 <https://github.com/tier4/scenario_simulator_v2/issues/304>`_ from tier4/feature/synchronize_clock
* enable synchronize timestamp
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Contributors: Kazuki Miyahara, Masaya Kataoka, Tatsuya Yamasaki

0.0.1 (2021-05-12)
------------------
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* remove unused header
* enable pass current time while attaching sensor
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* apply reformat
* update include guard
* rename package
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit
