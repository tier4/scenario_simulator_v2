^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cpp_mock_scenarios
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

16.4.2 (2025-05-23)
-------------------

16.8.3 (2025-07-24)
-------------------
* Merge branch 'master' into patch-1
* Merge branch 'master' into patch-1
* Merge branch 'master' into patch-1
* Contributors: Kotaro Yoshimoto

16.8.2 (2025-07-24)
-------------------

16.8.1 (2025-07-22)
-------------------
* Merge branch 'master' into fix-orientation-availability
* Contributors: Kotaro Yoshimoto

16.8.0 (2025-07-22)
-------------------
* Merge pull request `#1625 <https://github.com/tier4/scenario_simulator_v2/issues/1625>`_ from tier4/feat/cpp_mock_scenarios_awsim_support
  CppMockScenarios AWSIM support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* fix(CppMockScenarios) Added map_path parameter to load a path to lanelet2 file when runing a scenario with AWSIM
* fix(CppMockScenarios) Added ego_model to load appropriet asset-key when run scenarios with AWSIM
* fix(CppMockScenarios) Added ego_model to load appropriet asset-key when run scenarios with AWSIM
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* fix(CppMockScenario) Fix issue with comment
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* ref(CppMockScenarios): Handle  map_path parameter in configure method
* ref(CppScenarioNode): Refactor spawnEgoEntity to unify goal assignemnt
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* ref(CppMockScenarios): Refactor spawnEgoEntity methods
* feat(CppMockScenarios): Add spawningEgoEntity without requesting a path generation
* fix(CppMockScenarios): Revert mock_test.launch.py changes
* fix(CppMockScenario) : Allow map_path override via ROS2 parameter when empty in constructor
* feat(CppMockScenario) Add support for 'map_path' launch argument
  - Added map_path as a launch argument with default empty string
* feat: add ego_model launch argument for specifying AWSIM EGO 3D model
  - Added a new launch argument `ego_model` with default value `""` to support specifying the 3D asset name for the EGO vehicle model in AWSIM.
  Supported values include: 'jpntaxi', 'jpntaxi20', 'bydj6', 'bydj6_gen2', 'lexus_rx450h'.
* feat(CppMockScenarios) Add AWSIM support
  - Added 'ego_model' to support spawninig specific ego model in AWSIM.
  - Introduced optional delayed scenario clock start (api\_.startNpcLogic()) using start(false).
* Contributors: Robotec, SzymonParapura, Taiga

16.7.6 (2025-07-15)
-------------------
* Merge branch 'master' into fix-orientation-availability
* Contributors: Kotaro Yoshimoto

16.7.5 (2025-07-11)
-------------------
* Merge branch 'master' into feature/CODEOWNERS
* Contributors: Kotaro Yoshimoto

16.7.4 (2025-07-11)
-------------------

16.7.3 (2025-07-08)
-------------------

16.7.2 (2025-07-07)
-------------------
* Merge branch 'master' into fix/zmqpp
* Merge branch 'master' into fix/zmqpp
* Contributors: Kotaro Yoshimoto

16.7.1 (2025-07-04)
-------------------
* Merge pull request `#1631 <https://github.com/tier4/scenario_simulator_v2/issues/1631>`_ from tier4/refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Merge branch 'master' into refactor/cmakelists
* Add missing ament_cmake_auto
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR

16.7.0 (2025-07-03)
-------------------
* Merge branch 'master' into feature/record_option_for_rosbag
* Merge branch 'master' into feature/record_option_for_rosbag
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

16.6.1 (2025-07-02)
-------------------

16.6.0 (2025-07-01)
-------------------
* Merge branch 'master' into feature/perception_noise_config_v3
* Merge branch 'master' into feature/perception_noise_config_v3
* Contributors: Kotaro Yoshimoto

16.5.11 (2025-06-26)
--------------------
* fix test case
* Merge branch 'master' into fix/lanelet_pose_in_do_nothing
* modify threashold
* set torelance
* Contributors: Masaya Kataoka

16.5.10 (2025-06-23)
--------------------
* Merge branch 'master' into devin/1750224079-update-communication-docs
* Contributors: Kotaro Yoshimoto

16.5.9 (2025-06-23)
-------------------
* Merge branch 'master' into feature/use-add-pr-comment
* Contributors: Kotaro Yoshimoto

16.5.8 (2025-06-23)
-------------------

16.5.7 (2025-06-18)
-------------------
* Merge branch 'master' into dependabot/pip/requests-2.32.4
* Contributors: Masaya Kataoka

16.5.6 (2025-06-12)
-------------------
* Merge branch 'master' into add-start-trigger-context
* Merge branch 'master' into add-start-trigger-context
* Contributors: Kotaro Yoshimoto

16.5.5 (2025-06-10)
-------------------
* Merge branch 'master' into refactor/scenario_test_runner
* Merge branch 'master' into refactor/scenario_test_runner
* Merge branch 'master' into refactor/scenario_test_runner
* Merge branch 'master' into refactor/scenario_test_runner
* Merge branch 'master' into refactor/scenario_test_runner
* Merge branch 'master' into refactor/scenario_test_runner
* Contributors: Kotaro Yoshimoto

16.5.4 (2025-06-06)
-------------------
* Merge branch 'master' into fix/non-symlink-install-sun
* Contributors: Kotaro Yoshimoto

16.5.3 (2025-06-06)
-------------------
* Merge branch 'master' into refactor/behavior_tree_4
* Merge branch 'master' into refactor/behavior_tree_4
* Merge branch 'master' into refactor/behavior_tree_4
* Contributors: Kotaro Yoshimoto, Taiga

16.5.2 (2025-06-04)
-------------------

16.5.1 (2025-06-03)
-------------------
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, ぐるぐる

16.5.0 (2025-06-03)
-------------------
* Merge branch 'master' into feature/set_route
* Merge branch 'master' into feature/set_route
* Merge branch 'master' into feature/set_route
* Contributors: Kotaro Yoshimoto

16.4.6 (2025-06-02)
-------------------

16.4.5 (2025-05-30)
-------------------

16.4.4 (2025-05-28)
-------------------
* Merge pull request `#1609 <https://github.com/tier4/scenario_simulator_v2/issues/1609>`_ from tier4/feature/delay_curventure_calculation
  Feature/delay curventure calculation
* Merge remote-tracking branch 'origin/master' into feature/delay_curventure_calculation
* Merge branch 'master' into feature/delay_curventure_calculation
* Merge remote-tracking branch 'origin/master' into feature/delay_curventure_calculation
* Merge remote-tracking branch 'origin/master' into feature/speed_up
* add enable_perf option
* Contributors: Masaya Kataoka, Taiga

16.4.3 (2025-05-27)
-------------------
* Merge branch 'master' into refactor/behavior-tree-2
* Bump version of scenario_simulator_v2 from version version 16.4.1 to version 16.4.2
* Merge branch 'master' into refactor/behavior-tree-2
* Contributors: Kotaro Yoshimoto, Taiga

16.4.1 (2025-05-23)
-------------------
* Merge branch 'master' into refactor/behavior-tree-1
* Merge branch 'master' into refactor/behavior-tree-1
* Contributors: Taiga

16.4.0 (2025-05-22)
-------------------
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Contributors: Kotaro Yoshimoto

16.3.11 (2025-05-21)
--------------------
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Merge commit '2be47bbd1a1a69ba584d2a37b11b3140e40f5f3d' into refactor/lanelet_matching
* Contributors: Koki Suzuki, Masaya Kataoka, koki suzuki

16.3.10 (2025-05-20)
--------------------
* Merge branch 'master' into fix/agnocastpreload
* Merge branch 'master' into dependabot/pip/setuptools-78.1.1
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

16.3.9 (2025-05-20)
-------------------

16.3.8 (2025-05-19)
-------------------

16.3.7 (2025-05-15)
-------------------
* Merge branch 'master' into fix/mics-objects-model3d
* Merge branch 'master' into fix/mics-objects-model3d
* Contributors: Tatsuya Yamasaki

16.3.6 (2025-05-14)
-------------------
* Merge branch 'master' into feature/arm64-buildtest
* Merge branch 'master' into feature/arm64-buildtest
* Contributors: Kotaro Yoshimoto, ぐるぐる

16.3.5 (2025-05-12)
-------------------
* Merge branch 'master' into fix/missing-rviz-config-and-npc-start
* Contributors: SzymonParapura

16.3.4 (2025-05-12)
-------------------

16.3.3 (2025-05-02)
-------------------

16.3.2 (2025-04-25)
-------------------
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Contributors: Grzegorz Maj, Masaya Kataoka

16.3.1 (2025-04-25)
-------------------
* Merge branch 'master' into fix/concealer-7/transition
* Merge branch 'master' into fix/concealer-7/transition
* Merge branch 'master' into fix/concealer-7/transition
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

16.3.0 (2025-04-25)
-------------------
* Merge branch 'master' into feature/agnocast
* Merge branch 'master' into feature/agnocast
* Revert "Revert "Merge branch 'master' into feature/agnocast""
  This reverts commit b54960a3492c52964556d54d5943c00cdfa10f50.
* Revert "Merge branch 'master' into feature/agnocast"
  This reverts commit a01992c8e365edd59a52d918cccfec61885234f2, reversing
  changes made to 7f892377f23e4a7bfec460cbfa9f7cdd1b644806.
* Merge branch 'master' into feature/agnocast
* Merge branch 'master' into feature/agnocast
* Merge branch 'master' into feature/agnocast
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk

16.2.0 (2025-04-24)
-------------------

16.1.4 (2025-04-23)
-------------------
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* Contributors: Grzegorz Maj, Masaya Kataoka

16.1.3 (2025-04-21)
-------------------
* Merge branch 'master' into fix/concealer-7
* Merge branch 'master' into fix/concealer-7
* Merge branch 'master' into fix/concealer-7
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

16.1.2 (2025-04-18)
-------------------
* Merge branch 'master' into docs/fix_architecture_type
* Merge branch 'master' into docs/fix_architecture_type
* Merge branch 'master' into docs/fix_architecture_type
* Contributors: Dawid Moszyński, Masaya Kataoka

16.1.1 (2025-04-18)
-------------------
* Merge branch 'master' into fix/magic_subscription_data_race
* Merge branch 'master' into fix/magic_subscription_data_race
* Contributors: Kotaro Yoshimoto, SzymonParapura

16.1.0 (2025-04-18)
-------------------
* Merge branch 'master' into feature/pedestrian_awareness
* Merge branch 'master' into feature/pedestrian_awareness
* Merge branch 'master' into feature/pedestrian_awareness
* Merge branch 'master' into feature/pedestrian_awareness
* Merge branch 'master' into feature/pedestrian_awareness
* Contributors: Masaya Kataoka, Taiga

16.0.0 (2025-04-17)
-------------------
* Merge pull request `#1334 <https://github.com/tier4/scenario_simulator_v2/issues/1334>`_ from tier4/RJD-1057-remove-functions-forwarded-to-entity-base-refactor
  RJD-1057 (4/5): Remove non-API member functions: EntityManager’s member functions forwarded to EntityBase (2/2)
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-refactor' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'origin/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-with-middle
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base' into RJD-1057-remove-functions-forwarded-to-entity-base
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* ref(cpp_mock_scenarios): use returned spawn() pointer
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Mateusz Palczuk

15.1.3 (2025-04-16)
-------------------
* Merge branch 'master' into fix/concealer/engage
* Merge branch 'master' into fix/concealer/engage
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

15.1.2 (2025-04-16)
-------------------

15.1.1 (2025-04-14)
-------------------

15.1.0 (2025-04-14)
-------------------
* Merge branch 'master' into feature/parameter_override
* Merge branch 'master' into feature/parameter_override
* Contributors: Kotaro Yoshimoto

15.0.7 (2025-04-10)
-------------------
* Merge pull request `#1529 <https://github.com/tier4/scenario_simulator_v2/issues/1529>`_ from tier4/fix/scenario_name
  fix scenario name
* apply format
* Merge branch 'master' into fix/scenario_name
* Merge branch 'master' into fix/scenario_name
* fix scenario name
* Contributors: Masaya Kataoka

15.0.6 (2025-04-09)
-------------------

15.0.5 (2025-04-04)
-------------------
* Merge branch 'master' into fix-turn-indicator-report
* Merge branch 'master' into fix-turn-indicator-report
* Contributors: Kem (TiankuiXian), Kotaro Yoshimoto

15.0.4 (2025-04-03)
-------------------

15.0.3 (2025-04-03)
-------------------
* Merge branch 'master' into feature/remove-trajectory-subscription
* Merge remote-tracking branch 'tier4/master' into feature/remove-trajectory-subscription
* Contributors: Mateusz Palczuk

15.0.2 (2025-04-02)
-------------------
* Merge branch 'master' into dependabot/pip/jinja2-3.1.6
* Contributors: Masaya Kataoka

15.0.1 (2025-04-02)
-------------------
* Merge branch 'master' into feature/support-context-gamma-test
* Merge branch 'master' into feature/support-context-gamma-test
* Contributors: Masaya Kataoka, Taiga

15.0.0 (2025-03-31)
-------------------
* Merge pull request `#1551 <https://github.com/tier4/scenario_simulator_v2/issues/1551>`_ from tier4/refactor/get_lateral_distance
  HdMapUtils refactor `lanelet_wrapper::distance::lateralDistance`
* Merge branch 'master' into refactor/get_lateral_distance
* Merge branch 'master' into refactor/get_lateral_distance
* remove hdmap_utils from `lateralDistance` function
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

14.2.3 (2025-03-28)
-------------------
* Merge branch 'master' into refactor/concealer-7
* Merge branch 'master' into refactor/concealer-7
* Merge branch 'master' into refactor/concealer-7
* Merge remote-tracking branch 'origin/master' into refactor/concealer-7
* Merge remote-tracking branch 'origin/master' into refactor/concealer-7
* Contributors: Tatsuya Yamasaki, yamacir-kit

14.2.2 (2025-03-27)
-------------------
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'RJD1057/change-order-of-members' into RJD-1057/unify-spawn
* Contributors: Kotaro Yoshimoto, f0reachARR, ぐるぐる

14.2.1 (2025-03-27)
-------------------
* Merge remote-tracking branch 'origin/master' into RJD-1057/no-specific-param-in-manager
* Contributors: f0reachARR

14.2.0 (2025-03-26)
-------------------
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

14.1.0 (2025-03-25)
-------------------
* Merge branch 'master' into feature/revival_getStopLineIds
* Contributors: Taiga

14.0.3 (2025-03-24)
-------------------
* Merge remote-tracking branch 'origin/master' into refactor/concealer-6
* Merge branch 'master' into refactor/concealer-6
* Merge remote-tracking branch 'origin/master' into refactor/concealer-6
* Merge remote-tracking branch 'origin/master' into refactor/concealer-6
* Merge remote-tracking branch 'origin/master' into refactor/concealer-6
* Contributors: Tatsuya Yamasaki, yamacir-kit

14.0.2 (2025-03-19)
-------------------
* Merge remote-tracking branch 'origin/master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR, ぐるぐる

14.0.1 (2025-03-18)
-------------------
* Merge branch 'master' into feature/support-internal-repository
* Merge branch 'master' into feature/support-internal-repository
* Merge branch 'master' into feature/support-internal-repository
* Contributors: Masaya Kataoka, Taiga

14.0.0 (2025-03-14)
-------------------
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Contributors: yamacir-kit

13.0.0 (2025-03-14)
-------------------
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Contributors: Tatsuya Yamasaki

12.3.2 (2025-03-13)
-------------------
* Merge branch 'master' into feature/faster-template-instantiation
* Merge remote-tracking branch 'origin/master' into feature/faster-template-instantiation
* Merge remote-tracking branch 'origin/master' into feature/faster-template-instantiation
* Merge remote-tracking branch 'origin/master' into feature/faster-template-instantiation
* Merge branch 'master' into feature/faster-template-instantiation
* Merge remote-tracking branch 'origin/master' into feature/faster-template-instantiation
* Merge remote-tracking branch 'origin/master' into feature/faster-template-instantiation
* Contributors: Shota Minami, Tatsuya Yamasaki

12.3.1 (2025-03-13)
-------------------

12.3.0 (2025-03-12)
-------------------
* Merge branch 'master' into metrics_output
* Merge branch 'master' into metrics_output
* Merge branch 'master' into metrics_output
* Merge remote-tracking branch 'origin/master' into metrics_output
* Merge branch 'master' into metrics_output
* Merge branch 'master' into metrics_output
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

12.2.1 (2025-03-11)
-------------------
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

12.2.0 (2025-03-10)
-------------------
* Merge branch 'master' into feat/add_new_vehicle_model
* Merge branch 'master' into feat/add_new_vehicle_model
* Merge branch 'master' into feat/add_new_vehicle_model
* Merge branch 'master' into feat/add_new_vehicle_model
* Merge branch 'master' into feat/add_new_vehicle_model
* Contributors: Tatsuya Yamasaki

12.1.2 (2025-03-07)
-------------------
* Merge branch 'master' into xtk/loc-noise-exp
* Contributors: Tatsuya Yamasaki

12.1.1 (2025-03-07)
-------------------

12.1.0 (2025-03-05)
-------------------
* Merge branch 'master' into feature/simple_sensor_simulator/new-noise-model
* Merge branch 'master' into feature/simple_sensor_simulator/new-noise-model
* Merge remote-tracking branch 'origin/master' into feature/simple_sensor_simulator/new-noise-model
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki, yamacir-kit

12.0.2 (2025-03-04)
-------------------
* Merge branch 'master' into RJD-1057/reorgnize-ostream-helper
* Contributors: ぐるぐる

12.0.1 (2025-02-26)
-------------------
* Merge branch 'master' into feature/push-latest-docker-tag
* Contributors: Masaya Kataoka

12.0.0 (2025-02-25)
-------------------
* Merge pull request `#1533 <https://github.com/tier4/scenario_simulator_v2/issues/1533>`_ from tier4/refactor/lanelet_wrapper_bound
  HdMapUtils refactor lanelet_wrapper::lanelet_map::leftBound rightBound
* Merge branch 'master' into refactor/lanelet_wrapper_bound
* Merge branch 'master' into refactor/lanelet_wrapper_bound
* remove hdmap_utils from `distanceToLaneBound`
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

11.1.0 (2025-02-21)
-------------------
* Merge branch 'master' into feature/execution_time
* Merge remote-tracking branch 'origin/master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Merge branch 'master' into feature/execution_time
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

11.0.0 (2025-02-20)
-------------------
* Merge branch 'master' into refactor/lanelet_wrapper_route
* Contributors: Tatsuya Yamasaki

10.3.3 (2025-02-18)
-------------------
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge remote-tracking branch 'origin/master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Merge branch 'master' into refactor/simple_sensor_simulator/noise
* Contributors: Tatsuya Yamasaki, yamacir-kit

10.3.2 (2025-02-17)
-------------------

10.3.1 (2025-02-17)
-------------------
* Merge branch 'master' into fix/use-capital-as
* Contributors: Kotaro Yoshimoto

10.3.0 (2025-02-14)
-------------------
* Merge pull request `#1503 <https://github.com/tier4/scenario_simulator_v2/issues/1503>`_ from tier4/feature/publisher-with-customizable-randomizer
  Feature/publisher with customizable randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge remote-tracking branch 'origin/master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge branch 'master' into feature/publisher-with-customizable-randomizer
* Merge remote-tracking branch 'origin/master' into feature/publisher-with-customizable-randomizer
* Merge remote-tracking branch 'origin/master' into feature/publisher-with-customizable-randomizer
* Add launch argument `parameter_file_path` to `scenario_test_runner`
* Contributors: Tatsuya Yamasaki, yamacir-kit

10.2.0 (2025-02-14)
-------------------
* Merge branch 'master' into feature/rosbag_storage
* Merge branch 'master' into feature/rosbag_storage
* Merge branch 'master' into feature/rosbag_storage
* Contributors: Kotaro Yoshimoto

10.1.2 (2025-02-14)
-------------------

10.1.1 (2025-02-13)
-------------------
* Merge pull request `#1525 <https://github.com/tier4/scenario_simulator_v2/issues/1525>`_ from tier4/doc/comment_about_respawn_ego
  add comment about respawn_ego scenario
* Merge branch 'master' into doc/comment_about_respawn_ego
* add comment about respawn_ego scenario
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

10.1.0 (2025-02-12)
-------------------
* Merge pull request `#1496 <https://github.com/tier4/scenario_simulator_v2/issues/1496>`_ from tier4/fix/respawn-ego-test
  fix: mock test launch option
* Merge branch 'master' into fix/respawn-ego-test
* fix: add missing test
* fix: disable respawn mock test
* fix: unusual word
* fix: launch option
* fix: respawn ego mock test
* fix: add respawn ego test
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, satoshi-ota

10.0.0 (2025-02-07)
-------------------
* Merge pull request `#1500 <https://github.com/tier4/scenario_simulator_v2/issues/1500>`_ from tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
  [extra] RJD-1057 (3+/5): change getEntity return type to the reference
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Remove variables used only once
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* ref(cpp_mock_scenario): adapt to getEntity changes - return reference
* Contributors: Dawid Moszynski, Dawid Moszyński, Mateusz Palczuk, Tatsuya Yamasaki

9.4.0 (2025-02-06)
------------------
* Merge branch 'master' into feature/support-latest-autoware-message-type
* Merge branch 'master' into feature/support-latest-autoware-message-type
* Merge remote-tracking branch 'origin/master' into feature/support-latest-autoware-message-type
* Merge branch 'master' into feature/support-latest-autoware-message-type
* Contributors: Tatsuya Yamasaki, yamacir-kit

9.3.1 (2025-02-06)
------------------
* Merge branch 'master' into chore/delete-target-branch-filter
* Contributors: Masaya Kataoka

9.3.0 (2025-02-05)
------------------
* Merge remote-tracking branch 'origin/master' into feature/docker/traffic_simulator
* Contributors: Masaya Kataoka

9.2.0 (2025-02-05)
------------------
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, SzymonParapura

9.1.0 (2025-02-04)
------------------
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Contributors: Dawid Moszyński, Grzegorz Maj, Kotaro Yoshimoto

9.0.3 (2025-01-31)
------------------
* Merge branch 'master' into RJD-1505/fix_slope_acceleration_sign
* Merge branch 'master' into RJD-1505/fix_slope_acceleration_sign
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into RJD-1505/fix_slope_acceleration_sign
* Contributors: Grzegorz Maj, Kotaro Yoshimoto

9.0.2 (2025-01-31)
------------------

9.0.1 (2025-01-31)
------------------
* Merge branch 'master' into feat/vel_model_acc
* Merge branch 'master' into feat/vel_model_acc
* Contributors: Kotaro Yoshimoto

9.0.0 (2025-01-30)
------------------
* Merge pull request `#1473 <https://github.com/tier4/scenario_simulator_v2/issues/1473>`_ from tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle
  RJD-1057 (3/5): Remove non-API member functions: EntityManager’s member functions forwarded to EntityBase (1/2)
* merge 8.0.2
* Merge tag '7.4.7' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'origin/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(traffic_simulator, cpp_mock_scenarios): rename isInPosition to isNerbyPosition
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(traffic_simulator): rename isEntitySpawned to isEntityExist
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'origin/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'origin/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'origin/RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(cpp_mock_scenarios): improve move_backward
* ref(cpp_mock_scenarios): improve overall
* ref(cpp_mock_scenarios): ref random001
* fix(cpp_mock_scenarios): merge changes in random001
* Merge remote-tracking branch 'origin/RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Adjust mock scenarios to new API
* Merge remote-tracking branch 'tier4/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Restore previous scenario condition
* Fix mock scenario with invalid entity
  resetBehaviorPlugin respawns entity so the pointer stored from before the action is invalid and new one should be obtained
* feat(simulator_core, api, entity_base, cpp_mock): move setEntityStatus to EntityBase, remove from api
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* feat(ego_entity, sumulator_core): remove asFieldOperatorApplication, develop getEgoEntity and dedicated methods in EgoEntity
* feat(api, entity_manager, cpp_mock): rename entityExist to isEntitySpawned, move checkCollision directly to API
* feat(entity_manager, behavior_tree, cpp_mock): remove getCurrentAction forwarding, set "waiting" as init action state in behavior_tree
* feat(entity_base, traffic_simulator, simulator_core): remove forwarding request*, move requestLaneChange to EntityBase
* feat(entity_base, traffic_simulator, simulator_core): remove forwarding setBehaviorParameter and setVelocityLimit
* feat(entity_base, traffic_simulator): remove forwarding setters to EntityBase, also some getters, left setVelocityLimit and setBehaviorParameter
* feat(entity_base, traffic_simulator): rename laneMatchingSucceed to isInLanelet, remove forwarding
* feat(entity_base, traffic_simulator): move reachPosition as isInPosition to EntityBase, remove forwarding
* feat(entity_base, traffic_simulator, cpp_mock): move isInLanelet to EntityBase, remove forwarding
* Merge remote-tracking branch 'origin/RJD-1056-remove-current-time-step-time' into RJD-1057-remove-functions-forwarded-to-entity-base
* feat(cpp_mock, traffic_simulator): change getEntity, use getEntityOrNullptr
* Remove forwarding of getStandStillDuration in EntityManager and API
* Remove forwarding of getCurrentAccel in EntityManager and API
* Remove forwarding of getCurrentTwist in EntityManager and API
* Remove forwarding of getEntityStatus in EntityManager and API
* Contributors: Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Mateusz Palczuk, robomic

8.0.2 (2025-01-28)
------------------
* Merge branch 'master' into RJD-1495/fix
* Merge branch 'master' into RJD-1495/fix
* Merge tag '7.4.7' into RJD-1495/fix
* avoid race condition by returning by value
* Contributors: Dawid Moszyński, Tatsuya Yamasaki, robomic

8.0.1 (2025-01-28)
------------------

8.0.0 (2025-01-24)
------------------
* Merge pull request `#1472 <https://github.com/tier4/scenario_simulator_v2/issues/1472>`_ from tier4/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
  HdMapUtils refactor (PR 1/6)  - create lanelet_wrapper: use ::lanelet_map and ::pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' of github.com:tier4/scenario_simulator_v2 into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge remote-tracking branch 'origin/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(cpp_scenario_mock): remove unused auto_sink variable
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge remote-tracking branch 'origin' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge remote-tracking branch 'origin' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge remote-tracking branch 'origin/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(traffic_simulator): improve Configuration, traffic_rules, lanelet_wrapper
* feat(cpp_mock_scenarios): adapt cpp screnarios for using pose:: from lanelet_wrapper
* Contributors: Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Mateusz Palczuk

7.4.7 (2025-01-20)
------------------
* Merge branch 'master' into RJD-1511/bug_fix
* Bump version of scenario_simulator_v2 from version 7.4.5 to version 7.4.6
* Merge branch 'master' into RJD-1511/bug_fix
* Merge branch 'master' into refactor/parameter_value_distribution
* Merge branch 'master' into refactor/parameter_value_distribution
* Contributors: Kotaro Yoshimoto, Michał Ciasnocha, Release Bot

* Merge branch 'master' into refactor/parameter_value_distribution
* Merge branch 'master' into refactor/parameter_value_distribution
* Contributors: Kotaro Yoshimoto

7.4.6 (2025-01-10)
------------------
* Merge remote-tracking branch 'origin/master' into dependabot/pip/jinja2-3.1.5
* Contributors: Masaya Kataoka

7.4.5 (2025-01-10)
------------------
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* Merge remote-tracking branch 'origin/master' into fix/pass_despawn_function_in_constructor
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* Contributors: Masaya Kataoka

7.4.4 (2025-01-09)
------------------
* Merge branch 'master' into refactor/concealer-2
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.4.3 (2025-01-07)
------------------
* Merge branch 'master' into tmp/pc-patch
* Contributors: Kotaro Yoshimoto

7.4.2 (2025-01-07)
------------------

7.4.1 (2024-12-24)
------------------
* Merge remote-tracking branch 'origin/master' into fix/canonicalize_function
* Merge remote-tracking branch 'origin/master' into fix/canonicalize_function
* Contributors: Masaya Kataoka

7.4.0 (2024-12-23)
------------------
* Merge pull request `#1464 <https://github.com/tier4/scenario_simulator_v2/issues/1464>`_ from tier4/RJD-1457/traffic_sink_refactor
  RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* style fix
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* code style
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* sink pedestrian test fix
* simplify auto_sink logic
* TrafficSinkConfig
* AutoSinkConfig
* review suggestions
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* TrafficSink refactor with despawn functionality
* Merge tag '6.0.1' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/enable_specify_entity_type_in_autosink
* fix check condition
* Merge branch 'master' into feature/enable_specify_entity_type_in_autosink
* use uint8_t instead of traffic_simulator_msgs::msg::EntityType
* remap debug marker
* modify launch file
* add testcase for autosink
* enable sink vehicle
* enable set traffic sink in cpp scenario
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

7.3.5 (2024-12-20)
------------------
* Merge branch 'master' into refactor/concealer-1
* Merge branch 'master' into refactor/concealer-1
* Merge branch 'master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.3.4 (2024-12-20)
------------------
* Merge branch 'master' into feature/is_in_intersection
* Merge remote-tracking branch 'origin/master' into feature/is_in_intersection
* Merge remote-tracking branch 'origin/master' into feature/is_in_intersection
* Contributors: Masaya Kataoka

7.3.3 (2024-12-18)
------------------

7.3.2 (2024-12-18)
------------------

7.3.1 (2024-12-17)
------------------
* Merge branch 'master' into fix/math-closest-point
* Merge branch 'master' into fix/math-closest-point
* Merge branch 'master' into fix/math-closest-point
* Merge branch 'master' into fix/math-closest-point
* Merge branch 'master' into fix/math-closest-point
* Contributors: Kotaro Yoshimoto

7.3.0 (2024-12-16)
------------------
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Contributors: Kotaro Yoshimoto, SzymonParapura

7.2.0 (2024-12-16)
------------------
* Merge branch 'master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge branch 'master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge branch 'master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge branch 'master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support' into RJD-736/autoware_msgs_support_and_localization_sim_mode_support
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.1.0 (2024-12-16)
------------------
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Merge branch 'master' into feature/time-to-collision-condition
* Merge remote-tracking branch 'origin/master' into feature/time-to-collision-condition
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.0.4 (2024-12-13)
------------------
* Merge branch 'master' into fix/speed-condition/backward-compatibility
* Merge remote-tracking branch 'origin/master' into fix/speed-condition/backward-compatibility
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.0.3 (2024-12-13)
------------------
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Contributors: Kotaro Yoshimoto

7.0.2 (2024-12-12)
------------------
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Contributors: Masaya Kataoka, Taiga

7.0.1 (2024-12-11)
------------------
* Merge branch 'master' into feature/act-starttrigger-optional
* Merge branch 'master' into feature/act-starttrigger-optional
* Contributors: Kotaro Yoshimoto, ぐるぐる

7.0.0 (2024-12-10)
------------------
* Merge pull request `#1454 <https://github.com/tier4/scenario_simulator_v2/issues/1454>`_ from tier4/RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* fix: replace "awf/universe" with "awf/universe/20240605" for architecture_type
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
  # Conflicts:
  #	simulation/traffic_simulator/src/traffic_lights/traffic_light_publisher.cpp
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
  # Conflicts:
  #	external/concealer/include/concealer/autoware.hpp
  #	external/concealer/include/concealer/autoware_universe.hpp
  #	external/concealer/include/concealer/field_operator_application_for_autoware_universe.hpp
  #	external/concealer/src/autoware_universe.cpp
  #	external/concealer/src/field_operator_application_for_autoware_universe.cpp
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* fix mock_test.launch.py to follow master
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge branch 'master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Fix cpp_mock_scenarios launch parameters
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/feature/manual_on_follow_trajectory' into feature/manual_on_follow_trajectory_not_auto
  # Conflicts:
  #	simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Michał Kiełczykowski

6.3.1 (2024-12-10)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-8-3
* Merge branch 'master' into fix/sonor-cloud-issue-8-3
* Contributors: Masaya Kataoka

6.3.0 (2024-12-10)
------------------
* Merge pull request `#1468 <https://github.com/tier4/scenario_simulator_v2/issues/1468>`_ from tier4/feature/lane-change-everywhere
* Merge branch 'master' into feature/lane-change-everywhere
* Merge remote-tracking branch 'origin/master' into feature/lane-change-everywhere
* replace true with false for include_opposite_direction option
* Merge remote-tracking branch 'origin/master' into feature/lane-change-everywhere
* Contributors: Kotaro Yoshimoto

6.2.5 (2024-12-09)
------------------
* Merge branch 'master' into fix/acc_by_slope
* Contributors: Kotaro Yoshimoto

6.2.4 (2024-12-09)
------------------
* Merge branch 'master' into refactor/speed-condition
* Merge branch 'master' into refactor/speed-condition
* Contributors: Tatsuya Yamasaki

6.2.3 (2024-12-05)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-8-2
* Contributors: Masaya Kataoka

6.2.2 (2024-12-04)
------------------
* Merge branch 'master' into refactor/distance-condition
* Merge remote-tracking branch 'origin/master' into refactor/distance-condition
* Contributors: Tatsuya Yamasaki, yamacir-kit

6.2.1 (2024-12-03)
------------------
* Merge branch 'master' into refactor/distance-condition-and-relative-distance-condition
* Contributors: Tatsuya Yamasaki

6.2.0 (2024-12-02)
------------------
* Merge branch 'master' into feature/relative-speed-condition
* Merge remote-tracking branch 'origin/master' into feature/relative-speed-condition
* Merge remote-tracking branch 'origin/master' into feature/relative-speed-condition
* Merge remote-tracking branch 'origin/master' into feature/relative-speed-condition
* Merge remote-tracking branch 'origin/master' into feature/relative-speed-condition
* Contributors: Tatsuya Yamasaki, yamacir-kit

6.1.3 (2024-11-29)
------------------
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/RJD-1057-traffic-lights-tests' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk, Tatsuya Yamasaki

6.1.2 (2024-11-29)
------------------
* Merge branch 'master' into refactor/interpreter
* Contributors: Tatsuya Yamasaki

6.1.1 (2024-11-29)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/sonarcloud_warning
* Merge branch 'master' into fix/sonarcloud_warning
* Merge branch 'master' into fix/sonarcloud_warning
* Contributors: Masaya Kataoka

6.1.0 (2024-11-29)
------------------

6.0.1 (2024-11-27)
------------------

6.0.0 (2024-11-27)
------------------
* Merge pull request `#1458 <https://github.com/tier4/scenario_simulator_v2/issues/1458>`_ from tier4/refactor/add_routing_graph_argument
* Merge branch 'master' into refactor/add_routing_graph_argument
* chore: fix build error
* Merge branch 'master' into refactor/add_routing_graph_argument
* Contributors: Kotaro Yoshimoto

5.5.0 (2024-11-27)
------------------

5.4.0 (2024-11-26)
------------------
* Merge branch 'master' into feature/shoulder_routing_graph
* Contributors: Kotaro Yoshimoto

5.3.4 (2024-11-21)
------------------
* Merge branch 'master' into fix/find_nearest_segment_index
* Contributors: Kotaro Yoshimoto

5.3.3 (2024-11-21)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-7
* Contributors: Masaya Kataoka

5.3.2 (2024-11-18)
------------------
* Merge branch 'master' into fix/interpreter/assign-route-action
* Merge branch 'master' into fix/interpreter/assign-route-action
* Contributors: Tatsuya Yamasaki

5.3.1 (2024-11-18)
------------------
* Merge branch 'master' into refactor/routing_graph
* Merge branch 'master' into refactor/routing_graph
* Contributors: Kotaro Yoshimoto

5.3.0 (2024-11-18)
------------------
* Merge branch 'master' into feature/manual_on_follow_trajectory_with_new_state
* Merge branch 'master' into feature/manual_on_follow_trajectory_with_new_state
* Merge branch 'master' into feature/manual_on_follow_trajectory_with_new_state
* Merge branch 'master' into feature/manual_on_follow_trajectory_with_new_state
* Merge remote-tracking branch 'origin/master' into feature/manual_on_follow_trajectory_with_new_state
* Merge remote-tracking branch 'origin/master' into feature/manual_on_follow_trajectory_with_new_state
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge remote-tracking branch 'origin/master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge remote-tracking branch 'origin/feature/manual_on_follow_trajectory' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge remote-tracking branch 'origin/master' into feature/manual_on_follow_trajectory
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

5.2.3 (2024-11-18)
------------------

5.2.2 (2024-11-15)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-6
* Contributors: Taiga

5.2.1 (2024-11-14)
------------------
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Contributors: Grzegorz Maj

5.2.0 (2024-11-14)
------------------
* Merge branch 'master' into feature/by_object_type
* Merge branch 'master' into feature/by_object_type
* Merge branch 'master' into feature/by_object_type
* Merge branch 'master' into feature/by_object_type
* Contributors: Tatsuya Yamasaki

5.1.1 (2024-11-13)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-5
* Merge branch 'master' into fix/sonor-cloud-issue-5
* Merge branch 'master' into fix/sonor-cloud-issue-5
* Contributors: Masaya Kataoka, Taiga

5.1.0 (2024-11-12)
------------------
* Merge pull request `#1357 <https://github.com/tier4/scenario_simulator_v2/issues/1357>`_ from tier4/feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* feat: support awf/universe/20240605 as architecture_type in mock_test.launch.py
* Merge remote-tracking branch 'origin/master' into feature/traffic_light_group
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp
  #	simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
  #	simulation/traffic_simulator/src/traffic_lights/traffic_light_publisher.cpp
* Merge branch 'master' into feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* Merge branch 'master' into feature/traffic_light_group
* Contributors: Kotaro Yoshimoto

5.0.2 (2024-11-11)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue
* Merge branch 'master' into fix/sonor-cloud-issue
* Contributors: Masaya Kataoka, Taiga

5.0.1 (2024-11-11)
------------------

5.0.0 (2024-11-08)
------------------
* Merge pull request `#1406 <https://github.com/tier4/scenario_simulator_v2/issues/1406>`_ from tier4/RJD-1057-remove-traffic-lights-from-entity-manager
  RJD-1057 (1/5): Remove non-API member functions: EntityManager’s TrafficLight related member functions
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* feat(traffic_light_manager): use TrafficLightsBase and TrafficLights instead of TrafficLightsManager/Supervisor
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk, Tatsuya Yamasaki

4.5.0 (2024-11-07)
------------------
* Merge branch 'master' into chore/extend-npc-matching-distance
* Merge branch 'master' into chore/extend-npc-matching-distance
* Contributors: Kotaro Yoshimoto

4.4.1 (2024-11-07)
------------------
* Merge pull request `#1404 <https://github.com/tier4/scenario_simulator_v2/issues/1404>`_ from tier4/RJD-1336/fix_request_speed_change
  RJD-1336/fix_request_speed_change_throws
* update RequestSpeedChangeRelativeScenario
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* Merge branch 'RJD-1336/fix_request_speed_change' of github.com:tier4/scenario_simulator_v2 into RJD-1336/fix_request_speed_change
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* add update frame to update other_statuses
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.4.0 (2024-11-07)
------------------
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'fix/longitudinal_distance' of github.com:tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'fix/longitudinal_distance' of github.com:tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.3.27 (2024-11-07)
-------------------

4.3.26 (2024-11-06)
-------------------

4.3.25 (2024-11-05)
-------------------

4.3.24 (2024-11-01)
-------------------
* Merge branch 'master' into fix/remove-topic-logic
* Merge branch 'master' into fix/remove-topic-logic
* Contributors: Masaya Kataoka

4.3.23 (2024-11-01)
-------------------

4.3.22 (2024-10-31)
-------------------
* Merge branch 'master' into fix/improved-readability
* Contributors: Masaya Kataoka

4.3.21 (2024-10-31)
-------------------
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Contributors: Grzegorz Maj, Masaya Kataoka

4.3.20 (2024-10-31)
-------------------
* Merge branch 'master' into RJD-1335/requestSpeedChange
* Contributors: Grzegorz Maj

4.3.19 (2024-10-30)
-------------------

4.3.18 (2024-10-18)
-------------------
* Merge branch 'master' into feature/json/boost-json
* Merge branch 'master' into feature/json/boost-json
* Merge remote-tracking branch 'origin/master' into feature/json/boost-json
* Contributors: Kotaro Yoshimoto, f0reachARR, ぐるぐる

4.3.17 (2024-10-17)
-------------------
* Merge branch 'master' into fix/remove_warnings_from_sonarcloud
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/remove_warnings_from_sonarcloud
* Contributors: Masaya Kataoka

4.3.16 (2024-10-15)
-------------------

4.3.15 (2024-10-10)
-------------------
* Merge pull request `#1361 <https://github.com/tier4/scenario_simulator_v2/issues/1361>`_ from tier4/fix/RJD-1296-fix-random001-ego-issue
  fix(cpp_mock_scenarios, ego_entity_simulation): fix ego issue in random001, fix getCurrentPose()
* Merge branch 'master' into feature/faster-compilation
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Merge branch 'master' into feature/faster-compilation
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Merge branch 'master' into fix/RJD-1296-fix-random001-ego-issue
* fix(cpp_mock_scenario): fix ego issue - spawn,move,despawn
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Contributors: Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Shota Minami

4.3.14 (2024-10-10)
-------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/joblist-update-stand-still-duration
* Merge branch 'master' into feature/joblist-update-stand-still-duration
* Contributors: Masaya Kataoka

4.3.13 (2024-10-09)
-------------------
* Merge branch 'master' into feature/use-autoware-state
* Merge branch 'master' into feature/use-autoware-state
* Merge branch 'master' into feature/use-autoware-state
* Merge branch 'master' into feature/use-autoware-state
* Contributors: Kotaro Yoshimoto

4.3.12 (2024-10-09)
-------------------

4.3.11 (2024-10-07)
-------------------
* Merge branch 'master' into feature/jpblist-update-traveled-distance
* Merge branch 'master' into feature/jpblist-update-traveled-distance
* Contributors: Masaya Kataoka

4.3.10 (2024-10-03)
-------------------

4.3.9 (2024-10-03)
------------------
* Merge branch 'master' into test/cmake_flag_with_debug_and_relwithdebinfo
* Contributors: Masaya Kataoka

4.3.8 (2024-10-02)
------------------
* Merge branch 'master' into 1377/isInLanelet
* Merge branch 'master' into 1377/isInLanelet
* Contributors: Grzegorz Maj, Masaya Kataoka

4.3.7 (2024-09-27)
------------------
* Merge branch 'master' into feature/sonar_cloud
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/sonar_cloud
* Contributors: Masaya Kataoka

4.3.6 (2024-09-27)
------------------

4.3.5 (2024-09-27)
------------------
* Merge branch 'master' into feature/lcov
* Contributors: Masaya Kataoka

4.3.4 (2024-09-27)
------------------
* Merge branch 'master' into RJD-1201/fix_quick_start
* Contributors: SzymonParapura

4.3.3 (2024-09-26)
------------------
* Merge branch 'master' into fix/acquire-position-action
* Merge branch 'master' into fix/acquire-position-action
* Merge branch 'master' into fix/acquire-position-action
* Merge remote-tracking branch 'origin/master' into fix/acquire-position-action
* Merge remote-tracking branch 'origin/master' into fix/acquire-position-action
* Merge remote-tracking branch 'origin/master' into fix/acquire-position-action
* Contributors: Tatsuya Yamasaki, yamacir-kit

4.3.2 (2024-09-24)
------------------
* Merge branch 'master' into fix/imu_frame
* Merge branch 'master' into fix/imu_frame
* Contributors: Kotaro Yoshimoto

4.3.1 (2024-09-19)
------------------
* Merge branch 'master' into fix/fix-eigen-variable-definition
* Merge branch 'master' into fix/fix-eigen-variable-definition
* Merge branch 'master' into fix/fix-eigen-variable-definition
* Merge branch 'master' into fix/fix-eigen-variable-definition
* Contributors: Masaya Kataoka

4.3.0 (2024-09-19)
------------------
* Merge branch 'master' into RJD-1201/documentation_update
* Contributors: SzymonParapura

4.2.9 (2024-09-19)
------------------
* Merge branch 'master' into RJD-1197/distance
* Merge branch 'master' into RJD-1197/distance
* resolve conflict
* Merge branch 'master' into RJD-1197/distance
* Contributors: Michał Ciasnocha, robomic

4.2.8 (2024-09-18)
------------------

4.2.7 (2024-09-13)
------------------
* Merge pull request `#1379 <https://github.com/tier4/scenario_simulator_v2/issues/1379>`_ from tier4/fix/hard-coded-update-rate
  fix(mock): hard-coded update rate
* fix(mock): hard-coded update rate
* Contributors: Masaya Kataoka, satoshi-ota

4.2.6 (2024-09-13)
------------------
* Merge branch 'master' into RJD-1197/pose_module
* Contributors: Masaya Kataoka

4.2.5 (2024-09-12)
------------------
* Merge pull request `#1373 <https://github.com/tier4/scenario_simulator_v2/issues/1373>`_ from tier4/fix/colcon_build_error_furthermore
  fix: install add_cpp_mock_scenario_test.cmake first, or colcon build won't pass
* fix: install add_cpp_mock_scenario_test.cmake first, or build won't pass
* Contributors: Masaya Kataoka, XiaoyuWang0601

4.2.4 (2024-09-12)
------------------

4.2.3 (2024-09-11)
------------------
* Merge pull request `#1368 <https://github.com/tier4/scenario_simulator_v2/issues/1368>`_ from tier4/fix/mock-test-launch-test
  fix: mock test launch
* fix: global frame rate 30.0 -> 20.0
* fix: set default rviz config
* fix: missing param
* fix: use global timeout
* fix: make it possible to change hard-coded parameters
* fix: load necessary parameters
* Contributors: Masaya Kataoka, satoshi-ota

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
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
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
* ref(traffic_simulator): use getCanonicalizedStatus, remove getStatus
* feat(cpp_mock_scenarios): add isPedestrain and isVehicle - use it
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
* fix(cpp_mack_utils): adapt define_traffic_source scenarios to getEntity()
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
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
* Merge pull request `#1339 <https://github.com/tier4/scenario_simulator_v2/issues/1339>`_ from tier4/fix/ament_auto_package
  fix CONFIG_EXTRAS argument of ament_auto_package macro
* fix ament_auto_package macro
* Contributors: Masaya Kataoka

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
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/entity_selection
* Contributors: Shota Minami, Tatsuya Yamasaki

3.2.0 (2024-07-18)
------------------
* Merge pull request `#1323 <https://github.com/tier4/scenario_simulator_v2/issues/1323>`_ from tier4/fix/spawn_position_of_map_pose
  Fill x/y value when spawning entity in map frame.
* Merge remote-tracking branch 'origin/master' into fix/spawn_position_of_map_pose
* comment in entity_status.pose = pose;
* add test scenario for validation
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

3.1.0 (2024-07-16)
------------------
* Merge branch 'master' into autoware_lanelet2_extension
* Merge branch 'master' into autoware_lanelet2_extension
* Contributors: Tatsuya Yamasaki

3.0.3 (2024-07-12)
------------------
* Merge pull request `#1319 <https://github.com/tier4/scenario_simulator_v2/issues/1319>`_ from tier4/test/synchronized-action-kashiwanoha-map
  fix: Change map of synchronized action's test
* Merge branch 'master' into test/synchronized-action-kashiwanoha-map
* Changed map from simple_cross_map to kashiwanoha_map since bug has  been removed.
* Contributors: Masaya Kataoka, koki suzuki

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
* fix(cpp_mock_scenarios): adapt synchronized_action
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* feat(cpp_mock_scenarios): change constructLaneletPose to constructCanonicalizedLaneletPose
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
* fix(cpp_mock_scenarios): adapt traffic_source scenarios to new canonicalization approach
* Merge master->ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Revert "tmp"
  This reverts commit 6149b4cd77fa9e18ced8152c9ca0242228b5966f.
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* tmp
* fix(cpp_mock_scenarios): fix use consider_pose_by_road_slope
* ref(traffic_simulator): global improvements, comments, revert unnecessary changes
* ref(traffic_simulator): use only toMapPose and laneletLength from ::pose
* fix(cpp_mock_scenarios): fix respawn_ego
* feat(traffic_simulator): use consider_pose_by_road_slope as static variable in CanonicaliedLaneletPose
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* fix(cpp_mock_scenarios): fix accuracy in traveled_distance
* fix(cpp_mock_scenario): fix load do nothing plugin
* ref(simulator_core, ego_entity_simulation): improve strings
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(traffic_simulator, cpp_mock_scenarios): separate getLaneletLength and adapt entire code
* ref(cpp_mock_scenarios): simplify radndom001 lane change check
* fix(cpp_mock_scenarios): fix after ::pose refactor
* ref(traffic_simulator): separate getMapPoseFromRelativePose
* ref(cpp_mock_scenarios): fix canonicalize
* ref(traffic_simulator): tidy up constructCanonicalizedLaneletPose
* ref(traffic_simulator): improve setEntityManager - use ::pose, improve CanonicalizedEntityStatus
* ref(cpp_mock_scenario): remove canonicalize for spawn and setEntityStatus
* ref(cpp_mock_scenarios): adapt to separated pose::canonicalize() and getCanonicalizeLaneletPose
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(traffic_simulator): use toLaneletPose() from separated pose collection
* feat(traffic_simulator): transform PoseUtils to pose collection
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(pose): use separated toMapPose
* feat(pose): separate pose utils methods
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
* Merge pull request `#1307 <https://github.com/tier4/scenario_simulator_v2/issues/1307>`_ from tier4/hakuturu583/fix/remove/stl_comment
  Remove incorrect comment `// headers in STL`
* apply reformat
* Remove incorrect comment `// headers in STL`
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

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
* Merge pull request `#1234 <https://github.com/tier4/scenario_simulator_v2/issues/1234>`_ from tier4/feature/synchronized_action
  Feature/synchronized action
* Merge branch 'master' into feature/synchronized_action
* fix bug
* Update requestSynchronize function and added new test scenario.
* chore: Update requestSynchronize function to fix border distance calculation
* chore: Update requestSynchronize function signature to include target_speed parameter
* Merge commit 'c50d79fce98242d76671360029b97c166412e76f' into feature/synchronized_action
* Merge remote-tracking branch 'origin/master' into feature/synchronized_action
* Remove unnecessary include statement
* Fix spawn positions in synchronized_action.cpp
* Merge commit 'bf6a962e14e3e85627fca226574120cdba30080e' into feature/synchronized_action
* Update target lanelet poses in synchronized_action.cpp
* Merge commit 'bd366bce147e65d5991b62db333cf35153dd96fb' into feature/synchronized_action
* Refactor synchronized_action.cpp to remove unnecessary parameter in requestSynchronize()
* Add synchronized_action subdirectory and change return type of keepStepTime function
* Fix formatting in synchronized_action.cpp
* Merge commit 'b03fd92759845935be79f7ac32366848c78a2a66' into feature/synchronized_action
* Fix synchronization bug in entity_base.cpp
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronized_action
* Merge commit '45d42a79d92c370387749ad16c10665deb42e02c' into feature/synchronized_action
* Merge branch 'master' into feature/synchronized_action
* Merge commit '1ceb05c7206e163eb8214ceb68f5e35e7880d7a4' into feature/synchronized_action
* Merge commit 'f74901b45bbec4b3feb288c4ad86491de642f5ca' into feature/synchronized_action
* Merge commit '8a9b141aaf6cf5a58f537781a47f66e4c305cea3' into feature/synchronized_action
* Update package version and refactor reachPosition method
* Remove unnecessary code and include statements
* Merge branch 'master' into feature/synchronized_action
* Refactor synchronized action onUpdate method
* Add map package for simple cross map
* Merge commit '27266909414686613cea4f9aa17162d33ecf4668' into feature/synchronized_action
* Fix lanelet target pose in synchronized action
* Merge commit 'ada77d59ffd6545105e40e88e4ad50050062a3d6' into feature/synchronized_action
* Merge commit '253fa785573217ad3a6bde882724a9e35a0c99ed' into feature/synchronized_action
* Update entity_base.hpp and synchronized_action.cpp
* Update synchronized action behavior
* Update entity synchronization logic to consider acceleration limit
* Merge branch 'feature/synchronized_action' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronized_action
* Update target lanelet poses and velocities
* 途中経過
* Refactor synchronization logic and add new API method
* Disable building of C++ mock scenarios and update requestSynchronize function
* Made draft mock scenarios
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
* Merge pull request `#1275 <https://github.com/tier4/scenario_simulator_v2/issues/1275>`_ from tier4/feature/improve-ros-parameter-handling
  Feature: improve ROS parameter handling
* getParameter -> getROS2Parameter
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Revert changes adding parameter checking
  After thic change the code is functionally the same as in the beginning
* Merge branch 'master' into feature/improve-ros-parameter-handling
* ref(ParameterManager): rename to NodeParameterHandler, improve
* Apply API getParameter function where possible
* Contributors: Dawid Moszynski, Masaya Kataoka, Mateusz Palczuk

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
* change format
* fix
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
* ref(traffic_simulator, distance): rename from getters to noun function name
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator,distance): ref getDistanceToLaneBound
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator): transform DistanceUtils to distance namespace
* feat(distance): use separated getDistanceToBound
* feat(distance): use separated getLateral and getLongitudinal distances
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Tatsuya Yamasaki

1.18.0 (2024-05-24)
-------------------
* Merge pull request `#1231 <https://github.com/tier4/scenario_simulator_v2/issues/1231>`_ from tier4/feature/traffic-source
  Feature/traffic source
* Merge branch 'master' into feature/traffic-source
* Merge branch 'master' into feature/traffic-source
* Merge remote-tracking branch 'origin/master' into feature/traffic-source
* Remove comment from TrafficSource large scenario - the issue no longer exists
* Remove "headers in STL" comments
* Apply patched changes
* Merge branch 'master' into feature/traffic-source
* Fix scenarios
* Merge branch 'feature/traffic-source-scenarios' into feature/traffic-source
* Add cpp mock scenario that demonstrates the performance limitations
* Add high rate test
* Add TrafficSource scenarios
* Contributors: Masaya Kataoka, Mateusz Palczuk, Tatsuya Yamasaki

1.17.2 (2024-05-22)
-------------------

1.17.1 (2024-05-21)
-------------------
* Merge pull request `#1255 <https://github.com/tier4/scenario_simulator_v2/issues/1255>`_ from tier4/fix/visualization
  Fix/visualization
* update rviz config
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
* Merge pull request `#1198 <https://github.com/tier4/scenario_simulator_v2/issues/1198>`_ from tier4/feature/respawn-entity
  Feature/respawn entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* QoS of the subscriber in respawn_ego scenario changed to match the one used in the initial_pose_adaptor
* Code cleaning
* Merge remote-tracking branch 'origin-ssh/master' into feature/respawn-entity
* RespawnEgo scenario temporarly removed from build
* Respawn ego scenario test time adjustment
* CMakeList style
* Code cleaning
* Removing unnecessary changes in field_operator_application_for_autoware_universe
* Adapting respawn_ego scenario time to tests timeout
* Code cleaning
* Respawn scenario simplification
* Respawn logic moved to API
* RespawnEntity added
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
* Merge pull request `#1224 <https://github.com/tier4/scenario_simulator_v2/issues/1224>`_ from tier4/fix/remove_headers_in_stl_comment
  remove // headers in STL comment
* remove // headers in STL comment
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

1.11.0 (2024-04-02)
-------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Merge remote-tracking branch 'origin/master' into feature/arm_support
* Merge remote-tracking branch 'upstream/master' into feature/arm_support
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/arm_support
* Merge remote-tracking branch 'origin' into feature/arm_support
* Contributors: Masaya Kataoka, f0reachARR

1.10.0 (2024-03-28)
-------------------
* Merge branch 'master' into feature/simple_sensor_simulator/custom_noise
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
* Merge pull request `#1210 <https://github.com/tier4/scenario_simulator_v2/issues/1210>`_ from tier4/feature/reset_behavior_plugin
  Feature/reset behavior plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* fix test scenario
* enable reset behavior parameter
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge remote-tracking branch 'origin/master' into random-test-runner-docs-update
* Contributors: Masaya Kataoka, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki

1.8.0 (2024-03-25)
------------------
* Merge pull request `#1201 <https://github.com/tier4/scenario_simulator_v2/issues/1201>`_ from tier4/feature/set_behavior_parameter_in_object_controller
  Feature/set behavior parameter in object controller
* apply reformat
* add C++ test sceario
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_behavior_parameter_in_object_controller
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
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge branch 'master' into feature/ego_slope
* Contributors: Dawid Moszyński, Kotaro Yoshimoto

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
* Merge remote-tracking branch 'test/master' into feature/new_release
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
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Contributors: Paweł Lech, pawellech1, yamacir-kit

0.9.0 (2023-12-21)
------------------
* Merge pull request `#1139 <https://github.com/tier4/scenario_simulator_v2/issues/1139>`_ from tier4/fix/geometry-bug-fixes
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* Merge pull request `#1145 <https://github.com/tier4/scenario_simulator_v2/issues/1145>`_ from tier4/feature/random_scenario
* modify default sensor model
* change default vehicle model
* remove unused lambda function
* Merge branch 'feature/random_scenario' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* fix typo
* Merge remote-tracking branch 'origin/master' into feature/random_scenario
* Remove unnecessary comments
* Fix lanechange time constraint scenarios
* Merge branch 'experimental/merge-master' into feature/test-geometry-spline-subspline
* Merge remote-tracking branch 'tier4/master' into experimental/merge-master
* Merge remote-tracking branch 'origin/master' into feature/traffic_light_confidence
* remove debug line
* remove function object
* add spawn_nearby_ego entity
* Merge pull request `#1113 <https://github.com/tier4/scenario_simulator_v2/issues/1113>`_ from tier4/feature/doxygen
* fix compile error
* add namespace
* update namespace
* add concealer
* add spawn outside vehicle
* overwrite label from parameter
* enable set label
* rename scenario class
* add namespace
* rename scenario classes
* add namespace
* add cpp_mock_scenarios
* remove debug lines
* Merge remote-tracking branch 'origin' into feature/RJD-96_detail_message_scenario_failure
* remove sending route function
* remove sending route
* enable run scenario
* update scenario
* Merge branch 'master' into AJD-805/baseline_update_rebased
* add comment
* uncomment targets
* fix scenario
* randomize speed
* despawn stopped pedestrian
* add offset variance parameter
* add s variance
* enable clean up entity
* set bounds
* enable update parameter
* add parameters
* modify condition
* update scenario
* update scenario
* update scenario
* add random scenario
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge pull request `#1080 <https://github.com/tier4/scenario_simulator_v2/issues/1080>`_ from tier4/doc/add_comment_for_pr_1074
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into refactor/lanelet-id
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge pull request `#1087 <https://github.com/tier4/scenario_simulator_v2/issues/1087>`_ from tier4/feature/drop_galactic_support
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* remove workbound for galactic
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* add comment about PR `#1074 <https://github.com/tier4/scenario_simulator_v2/issues/1074>`_
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Mateusz Palczuk, Michał Kiełczykowski, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1074 <https://github.com/tier4/scenario_simulator_v2/issues/1074>`_ from tier4/fix/clock
* Fix some unit tests
* Lipsticks
* Remove member function `SimulationClock::initialize`
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1018 <https://github.com/tier4/scenario_simulator_v2/issues/1018>`_ from tier4/fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Canonicalize lanelet pose in cpp_mock_scenarios
* Merge branch 'master_fe8503' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_6789' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_4284' into fix/longitudinal_distance_fixed_master_merged
* cleanup code
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* apply reformat
* rename data type
* apply format in mock
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* rename data type
* apply format
* add namespace
* fix follow lane action
* fix compile errors in mock scenarios
* remove glog
* add glog to the mock scenario
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* fix compile error
* fix get longitudinal distance logic
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, kyoichi-sugahara, yamacir-kit

0.7.0 (2023-07-26)
------------------
* moved vehicle simulation to simple sensor simulator
* traffic light test (to be reverted)
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'master' into feature/rtc_custom_command_action
* Merge pull request `#1011 <https://github.com/tier4/scenario_simulator_v2/issues/1011>`_ from tier4/feature/do_nothing_plugin
* fix comment
* fix cmake lint error
* remove unused white line
* add description for test case
* add test case for loading do_nothing plugin
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#1004 <https://github.com/tier4/scenario_simulator_v2/issues/1004>`_ from tier4/feat/v2i_custom_command_action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* refactor(traffic_simulator): forward getTrafficLights function to each type of traffic lights
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#979 <https://github.com/tier4/scenario_simulator_v2/issues/979>`_ from RobotecAI/ref/AJD-696_clean_up_metics_traffic_sim
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#894 <https://github.com/tier4/scenario_simulator_v2/issues/894>`_ from tier4/fix/cleanup_code
* ref(cpp_mock_scenarios): remove metrics
* Merge remote-tracking branch 'origin/master' into clean-dicts
* ref(cpp_mock_scenarios): remove metrics
* ref(traffic_sim): remove metrics except out_of_range
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' into feature/noise_lost_object
* Merge pull request `#955 <https://github.com/tier4/scenario_simulator_v2/issues/955>`_ from tier4/mock-build-switching
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* remove WITH_INTEGRATION_TEST option
* feat(mock/CMake): add build switch
* Merge branch 'master' into feature/noise_lost_object
* Merge pull request `#951 <https://github.com/tier4/scenario_simulator_v2/issues/951>`_ from tier4/fix/warnings
* Merge branch 'master' into import/universe-2437
* Merge remote-tracking branch 'origin/master' into fix/warnings
* Merge pull request `#858 <https://github.com/tier4/scenario_simulator_v2/issues/858>`_ from tier4/feature/traveled_distance_as_api
* add distance mock
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Change boost::optional to std::optional
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* remove C++ warnings
* Merge pull request `#945 <https://github.com/tier4/scenario_simulator_v2/issues/945>`_ from tier4/feature/get_lateral_distance
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* update test cases
* add test case
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#909 <https://github.com/tier4/scenario_simulator_v2/issues/909>`_ from tier4/feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/feature/jerk_planning' into feature/interpreter/speed-profile-action
* add checking transition step for avoiding infinite loop
* fix scenario
* add new scenario
* remove debug line
* check target speed reached first
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* configure scenario
* configure request_speed_change_relative scenario
* configure request space change relative scenario
* configure merge_left scenario
* configure scenario
* enable check twist acceleration
* fix stop line mergin
* fix momenaty stop scenario
* fix calculate stop distance function
* add getRunningDistance function
* fix speed planning logic
* fix relative logic
* fix loop
* fix getCurrentTwist function
* add AUTO shape
* remove debug line
* enable run planing jerk
* fix plan function
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* add getQuadraticAccelerationDuration function
* add debug lines
* add check
* enable calculate jerk
* modify argument
* update mock
* fix jerk planning logic
* modify default value
* add default value
* add setter for acceleration/deceleration rate
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Replace boost::optional with std::optional
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/traveled_distance_as_api
* Remove unnecessary scenario test
* remove `TraveledDistanceScenario` from `cpp_mock_scenarios`
* Remove TraveledDistanceMetric
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Merge branch 'feature/reset_acecel_in_request_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Shota Minami, Tatsuya Yamasaki, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/behavior-parameter
* Merge pull request `#901 <https://github.com/tier4/scenario_simulator_v2/issues/901>`_ from tier4/feature/speed_action_with_time
* fix entity name
* add test scenario for relative
* add test scenario for time constraint
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* Merge branch 'master' into fix/interpreter/custom_command_action
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#898 <https://github.com/tier4/scenario_simulator_v2/issues/898>`_ from tier4/feature/interpreter/speed-profile-action
* Rename `DriverModel` to `BehaviorParameter`
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge pull request `#896 <https://github.com/tier4/scenario_simulator_v2/issues/896>`_ from tier4/refactor/traffic_simulator/spawn
* Update API::spawn (VehicleEntity) to receive position
* Update `API::spawn` (PedestrianEntity) to receive position
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* Update `API::spawn` (MiscObjectEntity) to receive position
* Merge branch 'master' into feature/interpreter/priority
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/concealer/acceleration
* Merge pull request `#823 <https://github.com/tier4/scenario_simulator_v2/issues/823>`_ from tier4/feature/start_npc_logic_api
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into fix/simple_sensor_simulator/fast_occupancy_grid
* remove boost::optional value
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge pull request `#854 <https://github.com/tier4/scenario_simulator_v2/issues/854>`_ from tier4/feature/remove_simple_metrics
* remove `Collision` and `StandstillDuration` from `cpp_mock_scenarios`
* Remove CollisionMetric and StandstillMetric
* Remove CollisionMetric and StandstillMetric
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'tier/master' into fix/obstacle_detection_raycaster
* Merge pull request `#836 <https://github.com/tier4/scenario_simulator_v2/issues/836>`_ from tier4/fix/trajectory_offset
* add getLaneletPose API
* fix problems in trajectory offset
* rename scenario
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#821 <https://github.com/tier4/scenario_simulator_v2/issues/821>`_ from tier4/feature/linelint
* fix lint error
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge pull request `#805 <https://github.com/tier4/scenario_simulator_v2/issues/805>`_ from tier4/doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Fix spells
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* Merge branch 'master' into fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* Merge pull request `#807 <https://github.com/tier4/scenario_simulator_v2/issues/807>`_ from tier4/feature/get_distance_to_lane_bound
* fix(cpp_mock_scenarios): modify build error in both galactic and humble
* modify scenario
* modify test scenario
* modify scenario
* add new scenario
* add getDistanceToLaneBound function
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Merge pull request `#778 <https://github.com/tier4/scenario_simulator_v2/issues/778>`_ from tier4/feature/zmqpp_vendor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Fix Licence
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* modify CMakeLists.txt
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/emergency_state_for_fault_injection
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Daisuke Nishimatsu, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/speed_up_get_length
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/traffic_light
* Merge pull request `#752 <https://github.com/tier4/scenario_simulator_v2/issues/752>`_ from tier4/feature/reset_acecel_in_request_speed_change
* remove unused if line in scenario
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge pull request `#740 <https://github.com/tier4/scenario_simulator_v2/issues/740>`_ from tier4/refactor/traffic_simulator/traffic_light_manager
* remove debug lines
* remove debug line
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Switch struct `TrafficLight` to experimental version
* Merge pull request `#749 <https://github.com/tier4/scenario_simulator_v2/issues/749>`_ from tier4/feature/job_system
* add test scenario
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Remove member function `setTrafficLightManager::set(Arrow|Color)Phase`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge pull request `#733 <https://github.com/tier4/scenario_simulator_v2/issues/733>`_ from tier4/feature/improve_ego_lane_matching
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' into fix/interpreter/interrupt
* change coordinate
* change goal position
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* rename data field
* fix compile error in catalog
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* rename to subtype
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Revert "[TMP] meassure update time"
* [TMP] meassure update time
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge branch 'master' into fix/interpreter/lifecycle
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge pull request `#671 <https://github.com/tier4/scenario_simulator_v2/issues/671>`_ from tier4/fix/lane_change_trajectory_shape
* update scenario
* Merge pull request `#670 <https://github.com/tier4/scenario_simulator_v2/issues/670>`_ from tier4/feature/request_speed_change_in_pedestrian
* Merge branch 'feature/request_speed_change_in_pedestrian' of https://github.com/tier4/scenario_simulator_v2 into fix/lane_change_trajectory_shape
* Merge remote-tracking branch 'origin/master' into feature/interpreter/lane-change-action
* enable set acceleration while walk straight state
* change speed linear in walk straight scenario
* remove debug message
* configure test scenario
* fix problem in passing driver model in pedestrian behavior plugin
* modify testcase and update release note
* Merge pull request `#669 <https://github.com/tier4/scenario_simulator_v2/issues/669>`_ from tier4/refactor/add_speed_change_namespace
* rename functions
* add speed_change namespace
* Merge remote-tracking branch 'origin/master' into feature/interpreter/speed-action
* Merge pull request `#664 <https://github.com/tier4/scenario_simulator_v2/issues/664>`_ from tier4/feature/lateral_velocity_constraint
* update test scenario
* add new test case
* enable change adaptive parameters
* add new test scenario
* modify scenario
* add test case for time constraint
* remove unused line
* update test scenario
* modify test scenario
* Merge pull request `#662 <https://github.com/tier4/scenario_simulator_v2/issues/662>`_ from tier4/fix/rename_trajectory
* rename data field and remove unused field
* Merge pull request `#661 <https://github.com/tier4/scenario_simulator_v2/issues/661>`_ from tier4/feature/lane_change_trajectory_shape
* Merge pull request `#654 <https://github.com/tier4/scenario_simulator_v2/issues/654>`_ from tier4/feature/request_relative_speed_change
* apply reformat
* remove debug line and modify scenario
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* add linear lanechange scenario
* merge fix/galactic_build
* add Lane change data types
* modify scenario
* modify test case
* set other status first
* add new test case
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into matsuura/feature/add-time-to-panel
* pull master
* merge master
* Merge tier4:master
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* remove glog from depends
* add glog and use unique_ptr
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#648 <https://github.com/tier4/scenario_simulator_v2/issues/648>`_ from tier4/feature/request_speed_change
* fix way of calling API
* fix compile error
* update scenario
* modify test case
* add test case
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: Piotr Zyskowski, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge pull request `#597 <https://github.com/tier4/scenario_simulator_v2/issues/597>`_ from tier4/refactor/traffic_simulator/spawning
* Merge branch 'master' into feature/interpreter/catalog
* Update `API::spawn` argument order
* Remove meaningless argument `is_ego` from some `spawn` overloads
* Update `API::spawn` to not to apply `setEntityStatus` to rest arguments
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.5.6 (2021-10-28)
------------------
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#566 <https://github.com/tier4/scenario_simulator_v2/issues/566>`_ from tier4/feature/behavior_plugin
* fix depends and LICENSE
* add debug output in mock node
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* fix plugin macro
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
* remove debug line
* remove boost::none check
* add spawnEntity function
* rename scenario class
* remove boost none in each metrics
* apply reformat
* remove spawn function without status
* remove unused depend
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#529 <https://github.com/tier4/scenario_simulator_v2/issues/529>`_ from tier4/feature/cpp_mock_scenario_ament_cmake
* pass double variable
* add test cases
* add timeout parameter
* change to comment out
* remove unused function
* remove cargo_delivery map
* add scenario_test_runner to the exec_depend
* add exec depend
* add WITH_INTEGRATION_TEST command
* add traffic_simulation_demo
* update workflow
* modify flag
* enable count number of failure/error/tests in junit
* add some test
* remove time
* enable pass ament_copyright
* add mock test
* enable output junit
* modify launch file
* add cmake function
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge pull request `#502 <https://github.com/tier4/scenario_simulator_v2/issues/502>`_ from RobotecAI/removed_cargo_delivery_dependency
* Merge remote-tracking branch 'origin/master' into feature/interpreter/distance-condition
* Merge pull request `#521 <https://github.com/tier4/scenario_simulator_v2/issues/521>`_ from tier4/feature/collision_metric
* Reorganize private road ele fix osm file
* Reverted adding parameters for cpp scenario node.
* Clang formatting
* Some missing changes in cpp mock scenarios
* Removed cargo_delivery dependency
* Merge pull request `#520 <https://github.com/tier4/scenario_simulator_v2/issues/520>`_ from tier4/feature/standstill_metric
* add test case
* Merge branch 'feature/standstill_metric' of https://github.com/tier4/scenario_simulator_v2 into feature/collision_metric
* add source
* update scenario
* add standstill duration scenario
* enable handle exception while calling updateFrame function
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Merge pull request `#512 <https://github.com/tier4/scenario_simulator_v2/issues/512>`_ from tier4/feature/test_entity
* change lanelet id
* modify scenario
* apply reformat
* add acquire position test cases
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Jaroszek, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge pull request `#507 <https://github.com/tier4/scenario_simulator_v2/issues/507>`_ from tier4/feature/add_scenario
* update lane assing logic for pedestrian
* apply reformat
* enable visualize position
* add LCOV_EXEL line
* add lane change scenarios to the workflow
* use direction
* fix test scenario
* add lanechange_right
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* fix typo in rviz
* fix typo
* fix typo of REVERSE
* fix typo of range
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* Merge pull request `#498 <https://github.com/tier4/scenario_simulator_v2/issues/498>`_ from tier4/feature/remove_unused_codes_in_entity
* remove pedestrian_parameters.hpp and vehicle_parameters.hpp
* enable generate pedestrian parameters without xml
* modify get parameters function
* enable get vehicle parameter without xml
* fix compile error
* modify catalog
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#487 <https://github.com/tier4/scenario_simulator_v2/issues/487>`_ from tier4/feature/get_longituninal_distance_behind
* add test cases for get longitudinal distance
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Rename launch-argument `with-rviz` to `launch_rviz`
* Feature/request acuire position in world coordinate (`#439 <https://github.com/tier4/scenario_simulator_v2/issues/439>`_)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Fix/offset calculation in lane coordinte (`#476 <https://github.com/tier4/scenario_simulator_v2/issues/476>`_)
* Merge remote-tracking branch 'origin/master' into fix/interpreter/misc
* Feature/metrics test (`#469 <https://github.com/tier4/scenario_simulator_v2/issues/469>`_)
* Feature/mock coverage (`#467 <https://github.com/tier4/scenario_simulator_v2/issues/467>`_)
* Feature/move backward action (`#461 <https://github.com/tier4/scenario_simulator_v2/issues/461>`_)
* Merge pull request `#459 <https://github.com/tier4/scenario_simulator_v2/issues/459>`_ from prybicki/patch-2
* Fix usage of uninitialized ZMQ API
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Masaya Kataoka, MasayaKataoka, Peter Rybicki, Wojciech Jaworski, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge pull request `#452 <https://github.com/tier4/scenario_simulator_v2/issues/452>`_ from tier4/fix/stop_at_crossing_entity_behavior
* add collision check
* add test scenario
* Merge pull request `#450 <https://github.com/tier4/scenario_simulator_v2/issues/450>`_ from tier4/fix/phase_control
* modify check condition
* modify success condition
* fix problems in TrafficLightManager::update() function
* Merge pull request `#448 <https://github.com/tier4/scenario_simulator_v2/issues/448>`_ from tier4/feature/add_cpp_scenarios
* add phase_control scenario
* add call thread::join in destructor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* add metrics scenario
* apply reformat
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/suppress_warnings
* Merge branch 'master' into namespace
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#433 <https://github.com/tier4/scenario_simulator_v2/issues/433>`_ from tier4/feature/yeild_to_merging_entity
* update decelerate and follow scenario
* add merge scenario
* add merge scenario
* fix to old version get longitudinal distance function
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/lane_change_npc_distance_in_lane_coordinate
* Merge remote-tracking branch 'origin/master' into namespace
* Merge pull request `#429 <https://github.com/tier4/scenario_simulator_v2/issues/429>`_ from tier4/feature/add_cpp_scenarios
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* apply reformat
* add stop function
* renam node
* fix lanelet path
* fix scenario
* fix compile errors
* add decelerate and follow scenario
* apply reformat
* add test scenario for following front entity
* change to enum value
* apply reformat
* enable colorlize output
* add termcolor
* remove lifecycle
* add error trigger
* remove sys.ext
* modify handler
* remove launch
* modify cmakelist.txt
* remove depends
* remove test dir
* add test
* update launch file
* add shutdown handler
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------

0.4.1 (2021-07-30)
------------------
* Merge pull request `#419 <https://github.com/tier4/scenario_simulator_v2/issues/419>`_ from tier4/feature/rename_moc_to_mock
* apply reformat
* Merge pull request `#418 <https://github.com/tier4/scenario_simulator_v2/issues/418>`_ from tier4/feature/add_cpp_scenario_node
* update docs
* move to mock package
* rename entity
* apply reformat and with_rviz argument
* remove view_kashiwanoha scenario
* add shutdown handler
* enable shutdown when fail or success
* add lanelet2 parameter
* add stop function
* fix compile errors
* add getparameters function
* addOnInitialize
* add configure function
* Merge pull request `#417 <https://github.com/tier4/scenario_simulator_v2/issues/417>`_ from tier4/feature/add_mock_scenarios
* apply reformat
* enable colorlize output
* add termcolor
* remove lifecycle
* add error trigger
* remove sys.ext
* modify handler
* remove launch
* modify cmakelist.txt
* remove depends
* remove test dir
* add test
* update launch file
* add shutdown handler
* Contributors: Masaya Kataoka

0.4.0 (2021-07-27)
------------------
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Update class Configuration to assert given map_path
* Add new struct 'Configuration' for class 'API'
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Contributors: Masaya Kataoka, yamacir-kit

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
* Merge pull request `#324 <https://github.com/tier4/scenario_simulator_v2/issues/324>`_ from tier4/feature/remove_unused_mock_package
* remove unused packages
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge remote-tracking branch 'origin/master' into feature/error-handling
* Contributors: Kazuki Miyahara, Masaya Kataoka, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#295 <https://github.com/tier4/scenario_simulator_v2/issues/295>`_ from tier4/fix/python_format
  reformat by black
* reformat by black
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* fix launch file
* use single quate
* add new line for the block
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* update namespace
* use clang_format
* apply reformat
* rename simulation_api package
* Merge branch 'master' into doc/zeromq
* Merge pull request `#225 <https://github.com/tier4/scenario_simulator_v2/issues/225>`_ from tier4/feature/support-autoware.iv-9
  Feature/support autoware.iv 9
* Update API class to receive scenario path as argument
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-8
* Merge pull request `#219 <https://github.com/tier4/scenario_simulator_v2/issues/219>`_ from tier4/feature/remove_xmlrpc
  Feature/remove xmlrpc
* add some library
* Merge branch 'master' into feature/support-autoware.iv-4
* Merge pull request `#207 <https://github.com/tier4/scenario_simulator_v2/issues/207>`_ from tier4/fix/xmlrpc_connection_lost
  Fix/xmlrpc connection lost
* modify parameter
* modify scenario
* modify scenario
* Merge branch 'master' into feature/support-autoware.iv-2
* Merge pull request `#201 <https://github.com/tier4/scenario_simulator_v2/issues/201>`_ from tier4/feature/publish_obstacle
  publish detection result
* Merge branch 'feature/publish_obstacle' of https://github.com/tier4/scenario_simulator.auto into feature/get_waypoint_from_autoware
* enable pass cpplint
* enable attach detection sensor
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#200 <https://github.com/tier4/scenario_simulator_v2/issues/200>`_ from tier4/feature/lidar_simulation
  Feature/lidar simulation
* fix typo
* add VLP32 model
* use helper function in mock
* update rviz
* update rviz
* enable update timestamp in sim
* enable attach lidar
* enable attach lidar via API
* fix problems in edge case
* modify mock
* remove marker publish thread
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#195 <https://github.com/tier4/scenario_simulator_v2/issues/195>`_ from tier4/feature/use_protobuf_in_spawn
  Feature/use protobuf in spawn
* enable pass compile
* Merge branch 'master' into feature/support-autoware.iv
* Merge branch 'master' into feature/awapi-accessor/non-awapi-topics
* Merge pull request `#188 <https://github.com/tier4/scenario_simulator_v2/issues/188>`_ from tier4/feature/idiot_npc
  Feature/idiot npc
* enable pass flake8
* enable spawn idiot NPC
* Merge branch 'master' into feature/custom-command-action
* Merge branch 'master' into feature/follow_odd_wg_upstream
* Merge pull request `#169 <https://github.com/tier4/scenario_simulator_v2/issues/169>`_ from tier4/feature/manual_drive
  Feature/manual drive
* configure directory
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, yamacir-kit
