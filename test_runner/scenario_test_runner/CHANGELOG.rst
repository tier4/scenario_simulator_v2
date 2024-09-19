^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scenario_test_runner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2024-04-18)
-------------------
* Merge pull request `#1223 <https://github.com/tier4/scenario_simulator_v2/issues/1223>`_ from tier4/refactor/drop_workflow
  Delete workflow feature and add alternative bash script
* chore: delete unnecessary diff
* Merge branch 'master' into refactor/drop_workflow
* fix: add dropped scenario to workflow.txt
* Merge remote-tracking branch 'origin/master' into refactor/drop_workflow
  # Conflicts:
  #	test_runner/scenario_test_runner/config/workflow_example.yaml
* refactor: rename workflow.csv to workflow.txt
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* chore: use remote workflow file(test)
* Merge branch 'master' into refactor/drop_workflow
* chore: revert scenario failure
* chore: test for scenario failure
* chore: update way to run workflow
* refactor: delete workflow.Workflow and rename workflow.py to scenario.py
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

2.5.0 (2024-07-08)
------------------
* Merge pull request `#1305 <https://github.com/tier4/scenario_simulator_v2/issues/1305>`_ from tier4/feature/publish_empty_context
  Feature/publish empty context
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* add GET_PARAMETER line
* update launch file
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
* Merge pull request `#1362 <https://github.com/tier4/scenario_simulator_v2/issues/1362>`_ from tier4/feature/ros2-parameter-forwarding
* Move function `make_vehicle_parameters` into function `make_parameters`
* Move parameter `use_sim_time` into function `make_parameters`
* Simplify collection of `autoware.` prefixed parameters
* Remove data member `traffic_simulator::Configuration::rviz_config_path`
* Add feature to forward parameters prefixed with `autoware.` to Autoware
* Contributors: Kotaro Yoshimoto, yamacir-kit

4.1.1 (2024-09-03)
------------------
* Merge pull request `#1207 <https://github.com/tier4/scenario_simulator_v2/issues/1207>`_ from tier4/fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* feat(use_sim_time): set default as false
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Revert "feat(params): set use_sim_time default as True"
  This reverts commit da85edf4956083563715daa3d60f0da1f94a423d.
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
* Merge pull request `#1348 <https://github.com/tier4/scenario_simulator_v2/issues/1348>`_ from tier4/fix/distance-with-lane-change
  Fix longitudinal dintance calculation with lane-change in `HdMapUtils::getLongitudinalDistance`
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* chore: add corner case to fix into a scenario
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

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
* Merge pull request `#1338 <https://github.com/tier4/scenario_simulator_v2/issues/1338>`_ from tier4/fix/interpreter/user-defined-value-condition
  Fix/interpreter/user defined value condition
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into fix/interpreter/user-defined-value-condition
* Update test scenario `ByValueCondition.UserDefinedValueCondition.yaml`
* Update test scenario `ByValueCondition.UserDefinedValueCondition.yaml`
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
* Merge pull request `#1316 <https://github.com/tier4/scenario_simulator_v2/issues/1316>`_ from tier4/relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* feat: add test scenarios for RelativeClearanceCondition to CI test
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Add ByEntityCondition.EntityCondition.RelativeClearanceCondition-back.yaml
* Update ByEntityCondition.EntityCondition.RelativeClearanceCondition.yaml to be a better test
* Update ByEntityCondition.EntityCondition.RelativeClearanceCondition.yaml to check RelativeClearanceCondition in more detail
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/relative-clearance-condition' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* feat: add ByEntityCondition.EntityCondition.RelativeClearanceCondition.yaml
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
* Merge pull request `#1325 <https://github.com/tier4/scenario_simulator_v2/issues/1325>`_ from tier4/feature/interpreter/lidar-configuration
  Feature/interpreter/lidar configuration
* Add a test scenario for `ObjectController`'s pseudo LiDAR property
* Contributors: Masaya Kataoka, yamacir-kit

3.3.0 (2024-07-23)
------------------
* Merge pull request `#1059 <https://github.com/tier4/scenario_simulator_v2/issues/1059>`_ from tier4/feature/interpreter/entity_selection
  Add `EntitySelection`
* Merge branch 'master' into feature/interpreter/entity_selection
* Update workflow.txt
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
* Fix typo
* Add test for LaneChangeAction
* Add test for TimeHeadwayCondition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Add test for ReachPositionCondition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Add test for (Relative)DistanceCondition
* Add test for StandStillCondition
* Add test for AccelerationCondition
* Add test scenario for EntitySelection
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
* fix(cpp_mock_scenarios): revert *-star changes
* fix(scenario): fix values in *RelativeDistanceConditionFreespace - after toCanonicalizedLaneletPose the exact values have changed (extremely small differences, but they affect the result)
* fix(scenario): move **-star scenario outside the lanelet - to avoid matching
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Tatsuya Yamasaki

2.6.0 (2024-07-08)
------------------
* Bump version of scenario_simulator_v2 from version 2.4.2 to version 2.5.0
* Merge pull request `#1305 <https://github.com/tier4/scenario_simulator_v2/issues/1305>`_ from tier4/feature/publish_empty_context
  Feature/publish empty context
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* add GET_PARAMETER line
* update launch file
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
* Merge branch 'master' into fix/remove_quaternion_operation
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
* fix launch
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, hakuturu583

1.15.5 (2024-05-07)
-------------------

1.15.4 (2024-05-01)
-------------------
* Merge pull request `#1237 <https://github.com/tier4/scenario_simulator_v2/issues/1237>`_ from tier4/feature/perf
  Feature/perf
* simplify launch file
* add make_launch_prefix function
* add enable_perf launch configuration
* add enable perf option
* Contributors: Kotaro Yoshimoto, hakuturu583

1.15.3 (2024-04-25)
-------------------
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* Contributors: Piotr Zyskowski

1.15.2 (2024-04-23)
-------------------
* Merge pull request `#1232 <https://github.com/tier4/scenario_simulator_v2/issues/1232>`_ from tier4/feature/update_default_architecture_type
  Update scenario_test_runner.launch.py
* Merge branch 'master' into feature/update_default_architecture_type
* Update scenario_test_runner.launch.py
  update default value of the `architecture_type`
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

1.15.1 (2024-04-18)
-------------------
* Merge branch 'master' into fix/occluded-object-in-grid
* Bump version of scenario_simulator_v2 from version 1.14.1 to version 1.15.0
* Merge branch 'master' into fix/occluded-object-in-grid
* Merge pull request `#1223 <https://github.com/tier4/scenario_simulator_v2/issues/1223>`_ from tier4/refactor/drop_workflow
  Delete workflow feature and add alternative bash script
* chore: delete unnecessary diff
* Merge branch 'master' into refactor/drop_workflow
* fix: add dropped scenario to workflow.txt
* Merge remote-tracking branch 'origin/master' into refactor/drop_workflow
  # Conflicts:
  #	test_runner/scenario_test_runner/config/workflow_example.yaml
* refactor: rename workflow.csv to workflow.txt
* Merge branch 'master' into refactor/drop_workflow
* Merge branch 'master' into refactor/drop_workflow
* chore: use remote workflow file(test)
* Merge branch 'master' into refactor/drop_workflow
* chore: revert scenario failure
* chore: test for scenario failure
* chore: update way to run workflow
* refactor: delete workflow.Workflow and rename workflow.py to scenario.py
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki, hakuturu583, ぐるぐる

1.14.1 (2024-04-12)
-------------------

1.14.0 (2024-04-12)
-------------------

1.13.0 (2024-04-11)
-------------------
* Merge pull request `#1216 <https://github.com/tier4/scenario_simulator_v2/issues/1216>`_ from tier4/feature/routing-algorithm
  Implement `DistanceCondition` / `RelativeDistanceCondition` for `shortest` of `RoutingAlgorithm`
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* chore: update ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* Merge remote-tracking branch 'origin/master' into feature/routing-algorithm
* doc: add clear names to conditions in ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* chore: add tests for RelativeDistanceCondition with shortest routing algorithm
* Merge branch 'master' into feature/routing-algorithm
* chore: update parameters in ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* chore: add test case for distance to backward point in adjacent lane
* fix: ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* chore: update ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml for distance conditions for freespace=true
* chore: update parameters in ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* chore: add ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml to scenario test
* chore: update parameters in ByEntityCondition.EntityCondition.DistanceCondition.Shortest.yaml
* Merge branch 'master' into feature/routing-algorithm
* chore: add test scenario for shortest routing algorithm
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
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/routing-algorithm
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, yamacir-kit

1.12.0 (2024-04-10)
-------------------
* Merge pull request `#1222 <https://github.com/tier4/scenario_simulator_v2/issues/1222>`_ from tier4/feature/user-defined-controller
  Feature/user defined controller
* Merge branch 'master' into feature/user-defined-controller
* Merge branch 'master' into feature/user-defined-controller
* Merge remote-tracking branch 'origin/master' into feature/user-defined-controller
* Update sample scenarios to specify controller name
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit

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
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Merge remote-tracking branch 'origin' into feature/arm_support
* add scenario
* Contributors: Masaya Kataoka, Ubuntu, f0reachARR

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
* Merge pull request `#1201 <https://github.com/tier4/scenario_simulator_v2/issues/1201>`_ from tier4/feature/set_behavior_parameter_in_object_controller
  Feature/set behavior parameter in object controller
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_behavior_parameter_in_object_controller
* fix workflow
* add test scenario
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

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
* chore: update scenario value to as is
* chore: enable flag defaultly
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* chore: revert scenario changes
* doc:
* chore: update scenario parameter
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge branch 'master' into feature/ego_slope
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* refactor: use consider_pose_by_road_slope instead of consider_lanelet_pose
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge branch 'master' into feature/ego_slope
* change default value
* update slop calculation logic
* modify launch file
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka

1.4.2 (2024-03-01)
------------------

1.4.1 (2024-02-29)
------------------

1.4.0 (2024-02-26)
------------------
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge branch 'master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Contributors: Dawid Moszyński

1.3.1 (2024-02-26)
------------------
* Merge pull request `#1195 <https://github.com/tier4/scenario_simulator_v2/issues/1195>`_ from tier4/feature/split_rviz_packages
  Feature/split rviz packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* move packages
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

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
* Merge pull request `#1182 <https://github.com/tier4/scenario_simulator_v2/issues/1182>`_ from tier4/feature/slope_vehicle_model
  Consider road slope in ego vehicle simulation
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/slope_vehicle_model
* chore: format
* fix: pass consider_acceleration_by_road_slope to inside of EgoEntitySimulator
* chore: format
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
  # Conflicts:
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* feat(scenario_test_runner): add consider_acceleration_by_road_slope for simple_sensor_simulator
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
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/new_release_flow
* Merge pull request `#1077 <https://github.com/tier4/scenario_simulator_v2/issues/1077>`_ from tier4/fix/autoware-shutdown
  Fix/autoware shutdown
* Merge branch 'master' into fix/autoware-shutdown
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/consider_tread_in_ego_entity
* Merge pull request `#1150 <https://github.com/tier4/scenario_simulator_v2/issues/1150>`_ from tier4/feature/real-time-factor-control
  Feature/real time factor control
* Merge remote-tracking branch 'tier/master' into feature/real-time-factor-control
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge pull request `#1159 <https://github.com/tier4/scenario_simulator_v2/issues/1159>`_ from tier4/revert/1096
  Revert/1096
* Changes after review
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into feature/real-time-factor-control
* use_sim_time for openscenario_interpreter is parameterized and False by default
* Merge branch 'tier4:master' into random-test-runner-docs-update
* Revert "chore: add example scenario of DeleteEntityAction"
  This reverts commit 8c6f8a498a226c2c35a9bedc18c90a95b5888f2b.
* Revert "chore: update author of sample scenario"
  This reverts commit 604127d61e0a3014e36f8b05af2c52ab44cd99ea.
* Setting concealer use_sim_time manually instead of using global arguments.
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Global real time factor set with launch argument fix
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Update `shutdownAutoware` to respect the parameter `sigterm_timeout`
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, pawellech1, yamacir-kit

0.9.0 (2023-12-21)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into prepare/release-0.9.0
* Merge pull request `#1129 <https://github.com/tier4/scenario_simulator_v2/issues/1129>`_ from tier4/feature/RJD-716_add_follow_waypoint_controller
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Merge pull request `#1149 <https://github.com/tier4/scenario_simulator_v2/issues/1149>`_ from tier4/feature/traffic-lights-awsim-support
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* fixed typo in scenario name
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* Merge pull request `#1033 <https://github.com/tier4/scenario_simulator_v2/issues/1033>`_ from tier4/feat/condition_groups_visualization
* add scenario file for varification
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* Merge branch 'feature/random_scenario' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* Merge remote-tracking branch 'origin/master' into feature/random_scenario
* v2i and conventional traffic lights AWSIM scenario samples
* Merge branch 'experimental/merge-master' into feature/test-geometry-spline-subspline
* ref(follow_trajectory): remove unecessary changes
* Merge pull request `#1126 <https://github.com/tier4/scenario_simulator_v2/issues/1126>`_ from tier4/fix/duplicated_nodes
* feat(follow_trajectory): add follow waypoint controller
* fix(launch): add comment about the change from LifecycleNode to Node
* Merge branch 'master' into fix/duplicated_nodes
* Merge pull request `#1111 <https://github.com/tier4/scenario_simulator_v2/issues/1111>`_ from tier4/feature/traffic_light_confidence
* Merge remote-tracking branch 'tier4/master' into experimental/merge-master
* fix(launch): remove name keyword from openscenario_interpreter, openscenario_prepreocessor and simples_sensor_simulator nodes
* Merge remote-tracking branch 'origin/master' into feature/traffic_light_confidence
* Merge pull request `#1121 <https://github.com/tier4/scenario_simulator_v2/issues/1121>`_ from tier4/fix/sign-of-relative-distance
* Merge remote-tracking branch 'origin/master' into fix/sign-of-relative-distance
* Merge pull request `#1096 <https://github.com/tier4/scenario_simulator_v2/issues/1096>`_ from tier4/feature/deleted-entity
* Update test scenario to not expect negative distance for `RelativeDistanceCondition` in non-lane coordinate systems
* feat: update CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1.yaml
* chore: update author of sample scenario
* chore: add example scenario of DeleteEntityAction
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into fix/rtc_command_action/continuous_execution
* Merge pull request `#1092 <https://github.com/tier4/scenario_simulator_v2/issues/1092>`_ from tier4/feature/control_rtc_auto_mode
* Merge pull request `#1099 <https://github.com/tier4/scenario_simulator_v2/issues/1099>`_ from tier4/pzyskowski/660/ss2-awsim-connection
* Merge pull request `#1098 <https://github.com/tier4/scenario_simulator_v2/issues/1098>`_ from tier4/fix/port_document
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* fix: destination of CustomCommandAction.RequestToCooperateCommandAction@v1.yaml
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1093 <https://github.com/tier4/scenario_simulator_v2/issues/1093>`_ from tier4/feature/RJD-614_follow_trajectory_action_pedestrian_cyclist_support
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* feat: add continuous rtc action execution in CustomCommandAction.RequestToCooperateCommandAction@v1.yaml
* refactor: delete cooperator property from CustomCommandAction.RequestToCooperateCommandAction@v1.yaml
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* change default port to 5555
* Merge remote-tracking branch 'origin/master' into fix/port_document
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* refactor(simulator_core): use featureIdentifiersRequiringExternalPermissionForAutonomousDecisions instead of manualModules
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1075 <https://github.com/tier4/scenario_simulator_v2/issues/1075>`_ from tier4/feature/RJD-96_detail_message_scenario_failure
* change default port to 8080
* Merge remote-tracking branch 'origin' into feature/RJD-96_detail_message_scenario_failure
* change port
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge branch 'master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge pull request `#1095 <https://github.com/tier4/scenario_simulator_v2/issues/1095>`_ from tier4/feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* code refactor
* implement freespace for relative distance condition
* Init working version of DistanceCondition freespace
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* feat: update scenario for new RTC feature
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* feat(test_runner): add FollowTrajectoryAction for bicycle and pedestrian
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* fix(test_runner): fix scenarios directory
* feat(scenario_test_runner): add workflow for simulation failure - detailed message
* Merge pull request `#1069 <https://github.com/tier4/scenario_simulator_v2/issues/1069>`_ from tier4/feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/drop_galactic_support
* Merge pull request `#1027 <https://github.com/tier4/scenario_simulator_v2/issues/1027>`_ from tier4/feature/new_traffic_light
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Initial version of freespace distance condition
* chore: change architecture_type to awf/universe/20230906
* misc object updated in sample awsim
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* updated npc in awsim_sample scenario
* Update `update` to use `updateStatus` instead of `setStatus`
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* added MiscObject to awsim_sample scenario
* fixed map path
* added modifier to awsim sample scenario
* sample awsim scenario added, model3d added to sample vehicle catalogue
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Add new test scenario `RoutingAction.FollowTrajectoryAction-autoware`
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* refactor: update architecture_type format
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* feat: add new architecture_type awf/universe/2023.08
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Mateusz Palczuk, Michał Kiełczykowski, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, f0reachARR, kyoichi-sugahara, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1024 <https://github.com/tier4/scenario_simulator_v2/issues/1024>`_ from tier4/feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* docs: add Property.detectedObjectGroundTruthPublishingDelay.yaml
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1056 <https://github.com/tier4/scenario_simulator_v2/issues/1056>`_ from tier4/feature/interpreter/sensor-detection-range
* merge lidar publishing delay
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1018 <https://github.com/tier4/scenario_simulator_v2/issues/1018>`_ from tier4/fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge pull request `#1030 <https://github.com/tier4/scenario_simulator_v2/issues/1030>`_ from tier4/feat/relative_object_position
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge pull request `#1031 <https://github.com/tier4/scenario_simulator_v2/issues/1031>`_ from tier4/feature/RJD-96_remove_verify_expected_result
* update scenario files
* revert lidar sensor delay's change
* merge master branch
* export scenario file from scenario editor
* update scenario file
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1054 <https://github.com/tier4/scenario_simulator_v2/issues/1054>`_ from tier4/remerge-1023
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Update y in test scenario
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Revert "Revert "Merge pull request `#1023 <https://github.com/tier4/scenario_simulator_v2/issues/1023>`_ from tier4/feat/pointcloud_delay""
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* fix cspell:disable
* Add reason for cspell disable
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Add cspell:disable-line in yaml
* Merge remote-tracking branch 'origin/master' into feat/relative_object_position
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* fix(workflow): remove failure.yaml scenarios
* Add test scenario
* feat(test_runner): remove expect in yamls
* feat(test_runner, openscenario_preprocessor): remove expect
* feat(openscenario_interpreter): remove intended_result
* Increase ReachPositionCondition tolerance distance
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_fe8503' into fix/longitudinal_distance_fixed_master_merged
* add interpreter for detection sensor range
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_6789' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_4284' into fix/longitudinal_distance_fixed_master_merged
* fix result checker script
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* fix some files
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* apply reformat
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Tatsuya Yamasaki, kosuke55, kyoichi-sugahara, yamacir-kit

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* fix(simple_sensor_simulator): provide vehicle parameters
* unified scneario test runner launch (added a way to not to launch simple_sensor simulator
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* brought back working version with SSS (break working with AWSIM)
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#906 <https://github.com/tier4/scenario_simulator_v2/issues/906>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge pull request `#1013 <https://github.com/tier4/scenario_simulator_v2/issues/1013>`_ from tier4/feature/rtc_custom_command_action
* fix: modify CustomCommandAction.RequestToCooperateCommandAction@v1.yaml
* Update accomplished to return false if it is the first call after start
* refactor:
* chore: add a scenario to test RequestToCooperateCommandAction@v1
* Remove follow clothoid and NURBS trajectory action
* Add some test scenarios
* Add scenario `FollowTrajectoryAction-star.yaml` to workflow_example
* Lipsticks
* Add scenario `FollowTrajectoryAction-straight.yaml` to workflow_example
* Update `FollowTrajectoryAction::accomplished` to work correctly
* Relax the condition for determining delay for the specified arrival time
* Revert "rerun autoware from launch"
* rerun autoware from launch
* working changes
* Improve time remaining calculation and speed planning
* Add some notes
* scenario test raunner awsim launch
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/rtc_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge pull request `#1007 <https://github.com/tier4/scenario_simulator_v2/issues/1007>`_ from tier4/documentation/interpreter/detected-object
* Fix typo
* Add new example `Property.detectedObjectMissingProbalibity.yaml`
* Add new example `Property.detectedObjectPositionStandardDeviation.yaml`
* Add new example `Property.detectedObjectPublishingDelay.yaml`
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#1004 <https://github.com/tier4/scenario_simulator_v2/issues/1004>`_ from tier4/feat/v2i_custom_command_action
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge pull request `#1002 <https://github.com/tier4/scenario_simulator_v2/issues/1002>`_ from tier4/feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* feat(scenario_test_runner): add test scenario of V2ITrafficSignalState
* Fix sample scenario to not to set initial speed to ego entity
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Update FollowPolylineTrajectoryAction to respect time limits, albeit imperfectly
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Update test scenario `RoutingAction.FollowTrajectoryAction` to Autoware independent
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Update `FollowPolylineTrajectoryAction` to work if `time` unspecified
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#980 <https://github.com/tier4/scenario_simulator_v2/issues/980>`_ from tier4/feature/interpreter/environment
* Merge branch 'master' into feature/interpreter/model3d-field
* Fix example scenario
* Fix scenario typo
* Add sample scenario of Environment
* Merge remote-tracking branch 'origin/master' into feature/interpreter/model3d-field
* Merge branch 'master' into feature/interpreter/environment
* Merge branch 'master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge branch 'master' into fix/cleanup_code
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/AJD-697_improve_port_management_zmq
* Merge pull request `#971 <https://github.com/tier4/scenario_simulator_v2/issues/971>`_ from tier4/feature/interpreter/delay_in_condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* Merge pull request `#978 <https://github.com/tier4/scenario_simulator_v2/issues/978>`_ from tier4/feature/interpreter/relative-heading-condition
* Merge remote-tracking branch 'origin/master' into clean-dicts
* Merge branch 'master' into feature/interpreter/model3d-field
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge branch 'master' into feature/noise_delay_object
* Merge remote-tracking branch 'origin/master' into feature/interpreter/relative-heading-condition
* Merge pull request `#975 <https://github.com/tier4/scenario_simulator_v2/issues/975>`_ from tier4/emergency-state/backwardcompatibility-1
* Merge branch 'master' into feature/noise_delay_object
* Update sample scenario to use one-argument version of `RelativeHeadingCondition`
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Add time-based testing scenaio for condtion edge
* Merge pull request `#972 <https://github.com/tier4/scenario_simulator_v2/issues/972>`_ from tier4/fix/openscenario_utility/conversion
* Revert "Revert some changes for `scenario_test_runner.py`"
* Revert some changes for `scenario_test_runner.py`
* Remove unreachable statements of `MacroExpander.__call_\_`
* Cleanup import statements of `scenario_test_runner.py`
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into fix/get-unique-route-lanelets
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#967 <https://github.com/tier4/scenario_simulator_v2/issues/967>`_ from RobotecAI/fix/AJD-655-terminates-sigint
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* refactor: apply ament_clang_format
* fix(test_runner):  ensure 0 exit code after sigint
* fix(test_runner):  ensure 0 exit code after sigint
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin' into fix/getting_next_lanelet
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#962 <https://github.com/tier4/scenario_simulator_v2/issues/962>`_ from tier4/feature/interpreter/relative-distance-condition
* Merge pull request `#954 <https://github.com/tier4/scenario_simulator_v2/issues/954>`_ from tier4/fix/python-installation
* Add some test scenario to `workflow_example`
* Update `DistanceCondition` to support lateral lane-coordinate distance
* Update `RelativeDistanceCondition` to support lateral lane-coordinate distance
* chore: add debug messages
* Add new test scenario `...RelativeDistanceCondition.yaml`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge branch 'master' into feature/noise_lost_object
* Cleanup
* Update package `scenario_test_runner` to use CMake to install scripts instead of setuptools
* Granted execute permission to some Python scripts
* Merge pull request `#952 <https://github.com/tier4/scenario_simulator_v2/issues/952>`_ from tier4/feature/interpreter/catalog
* Update scenario `sample.yaml` to use catalog `openscenario_experimental_catalog`
* Fix scenario `sample.yaml` to specify `value` of `Property` as string instead of number
* Merge pull request `#951 <https://github.com/tier4/scenario_simulator_v2/issues/951>`_ from tier4/fix/warnings
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* suppress warning from setuptools
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* Merge remote-tracking branch 'origin/master' into import/universe-2437
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#937 <https://github.com/tier4/scenario_simulator_v2/issues/937>`_ from tier4/feature/interpreter/noise
* Update `AssignControllerAction` to consider some object detection properties
* Merge branch 'master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#928 <https://github.com/tier4/scenario_simulator_v2/issues/928>`_ from tier4/feature/interpreter/speed-profile-action
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/speed-profile-action
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#909 <https://github.com/tier4/scenario_simulator_v2/issues/909>`_ from tier4/feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#915 <https://github.com/tier4/scenario_simulator_v2/issues/915>`_ from tier4/fix/add_on_exit_to_simulator
* Update `SpeedProfileAction` debug informations
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Update `SpeedProfileAction` to select `Transition` by attribute `followingMode`
* chore(test_runner): fix linter error
* feat(test_runner): use ShutdownOnce instead of Shutdown
* Update example scenario `SpeedProfileAction.yaml`
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* chore(launch): add on_exit flag to interpreter
* add on_exit flag to the simulator
* fix speed action scenrio
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Minami Shota, Shota Minami, Tatsuya Yamasaki, f0reachARR, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge pull request `#903 <https://github.com/tier4/scenario_simulator_v2/issues/903>`_ from tier4/feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge pull request `#883 <https://github.com/tier4/scenario_simulator_v2/issues/883>`_ from tier4/feature/interpreter/priority
* add a part for checking priority features to all-in-one.yaml
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* Merge branch 'master' into fix/interpreter/custom_command_action
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#898 <https://github.com/tier4/scenario_simulator_v2/issues/898>`_ from tier4/feature/interpreter/speed-profile-action
* Add test scenario `LongitudinalAction.SpeedProfileAction`
* Replace msg names to new ones
* Delete unused files
* add a lane change scenario to all-in-one.yaml
* Merge remote-tracking branch 'origin/master' into feature/interpreter/priority
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Extract openscenario_preprocessor
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* Merge branch 'master' into feature/interpreter/follow-trajectory-action-3
* Merge pull request `#890 <https://github.com/tier4/scenario_simulator_v2/issues/890>`_ from tier4/refactor/test_runner
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge remote-tracking branch 'origin/master' into feature/interpreter/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into refactor/test_runner
* Merge pull request `#892 <https://github.com/tier4/scenario_simulator_v2/issues/892>`_ from tier4/feature/interpreter/follow-trajectory-action-2
* Fix workflow execution
* Fix bug in lifecycle operation
* Integrate preprocessor into scenario_test_runner.py
* Add openscenario_preprocessor to launch file
* Format scenario_test_runner.py
* Add new test scenario `RoutingAction.FollowTrajectoryAction.yaml`
* Add empty implementation for new structure of test_runner
* Rename / Add annotations to functions in scenario_test_runner.py
* Code cleanup: delete yaml scenarios
* WIP: Implement (para)lellization and add scenarios
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/fix/ci_error' into feature/start_npc_logic_api
* Merge branch 'master' into feature/occupancy_grid_docs
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Revert change for scenario `sample.yaml`
* Rename CMake targets to `lanelet2_extension_psim*` from `lanelet2_extension*`
* Merge pull request `#847 <https://github.com/tier4/scenario_simulator_v2/issues/847>`_ from tier4/feature/value_constraint
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Fix sample.yaml
* Add value constraint to sample scenario
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into doc/6th_improvement
* Merge pull request `#837 <https://github.com/tier4/scenario_simulator_v2/issues/837>`_ from tier4/update/rviz_display
* Fix runtime errors
* Fix the way to import rviz config path
* Format files
* Format files
* Format files
* Merge remote-tracking branch 'tier/master' into fix/obstacle_detection_raycaster
* Merge branch 'master' into fix/trajectory_offset
* Add command line option to pass rviz_config
* Merge pull request `#830 <https://github.com/tier4/scenario_simulator_v2/issues/830>`_ from tier4/feature/interpreter/relative-heading-condition
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Add new NonStandardOperation `evaluateRelativeHeading`
* Update `UserDefinedValueCondition` to support function-style expression (EXPERIMENTAL)
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/arrange_docs_and_fix_copyright
* Fix Licence
* Merge pull request `#750 <https://github.com/tier4/scenario_simulator_v2/issues/750>`_ from tier4/fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into fix/autoware/reverse-gear
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Migrate README of scenario_test_runner to docs directory
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge pull request `#758 <https://github.com/tier4/scenario_simulator_v2/issues/758>`_ from tier4/feature/interpreter/instantaneously-transition
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* LifycycleNode's timeout is the same as global_timeout
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into feature/zmqpp_vendor
* Merge pull request `#785 <https://github.com/tier4/scenario_simulator_v2/issues/785>`_ from tier4/doc/improve
* Fix broken links
* Fix old "TierIV" annotation
* scenario_test_runner raises RuntimeError when openscenario_interpreter doesn't respond to a request
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Lipsticks
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* fail if autoware_launch_file doesn't exist
* Contributors: Adam Krawczyk, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge pull request `#740 <https://github.com/tier4/scenario_simulator_v2/issues/740>`_ from tier4/refactor/traffic_simulator/traffic_light_manager
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Remove enumeration `TrafficLightColor::NONE`
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* on_exit=Shutdown()
* reuse executor
* remove on_exit=Shtudown()
* extend sigterm_timeout
* shutdown_flag -> is_running
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#734 <https://github.com/tier4/scenario_simulator_v2/issues/734>`_ from tier4/fix/interpreter/global-action
* Remove unnecessary prints from the interpreter and simulator
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge pull request `#727 <https://github.com/tier4/scenario_simulator_v2/issues/727>`_ from tier4/feature/interpreter/reader
* Add new example node `uniform_distribution`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Add new experimental substitution `$(ros2 ...)`
* add disconect() to ~Interpreter(). stop zeromq call if shut down.
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/semantics
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into doc/release_note_format
* Merge pull request `#720 <https://github.com/tier4/scenario_simulator_v2/issues/720>`_ from tier4/refactor/interpreter/execution
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Update some sample scenarios to ommit optional element `TrafficSignals`
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge pull request `#717 <https://github.com/tier4/scenario_simulator_v2/issues/717>`_ from tier4/improvement/sample-scenario
  Improvement/sample scenario
* Fix some paths `test/scenario` to `scenario`
* Move directory `test/scenario` to `scenario`
* Update `sample` Vehicle parameter to match `sample_vehicle_description`
* Fix scenario `minimal` to work
* Remove some unused test scenarios
* Rename scenario `autoware-simple` to `sample`
* Update scenario `autoware-simple` to parameterize lane IDs
* Merge pull request `#716 <https://github.com/tier4/scenario_simulator_v2/issues/716>`_ from tier4/dependency/remove-lexus-description
  Remove `lexus_description` from dependency
* Update sample scenario `autoware-simple`
* Remove `lexus_description` from dependency
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* Merge pull request `#704 <https://github.com/tier4/scenario_simulator_v2/issues/704>`_ from tier4/feature/autoware-external-api
  Feature/autoware external api
* Replace `AwapiAutowareStatus` with `autoware_auto_system_msgs::msg::AutowareState`
* Merge pull request `#702 <https://github.com/tier4/scenario_simulator_v2/issues/702>`_ from tier4/fix/no-perform-method
  fix: no perform method in str object
* fix: no perform method in str object
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Remove architecture_type `awf/auto`
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#679 <https://github.com/tier4/scenario_simulator_v2/issues/679>`_ from tier4/refactor/interpreter/scope
  Refactor/interpreter/scope
* Lipsticks
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#672 <https://github.com/tier4/scenario_simulator_v2/issues/672>`_ from tier4/fix/interpreter/lifecycle
  Fix/interpreter/lifecycle
* construct/destruct connection at on_activate/on_deactivate
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Remove architecture_type `tier4/proposal`
* Contributors: Makoto Tokunaga, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change_in_pedestrian
* Merge branch 'master' into feature/request_speed_change_in_pedestrian
* Merge pull request `#668 <https://github.com/tier4/scenario_simulator_v2/issues/668>`_ from tier4/feature/interpreter/lane-change-action
* Add new test scenario `LateralAction.LaneChangeAction`
* Merge https://github.com/tier4/scenario_simulator.auto into feature/control_from_relation_id
* Merge pull request `#665 <https://github.com/tier4/scenario_simulator_v2/issues/665>`_ from tier4/feature/interpreter/speed-action
* Update scenario `LongitudinalAction.SpeedAction` to work on CI
* Add new test scenario `LongitudinalAction.SpeedAction`
* Update member function `SpeedAction::accomplished`
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
