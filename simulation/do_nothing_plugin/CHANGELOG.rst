^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package do_nothing_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2023-12-21)
------------------
* Merge branch 'feature/random_scenario' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* Merge remote-tracking branch 'origin/master' into feature/random_scenario
* Merge branch 'master' into fix/duplicated_nodes
* Merge pull request `#1111 <https://github.com/tier4/scenario_simulator_v2/issues/1111>`_ from tier4/feature/traffic_light_confidence
* chore: suppress warning from boost library
* Merge remote-tracking branch 'origin' into feature/RJD-96_detail_message_scenario_failure
* Merge branch 'master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1090 <https://github.com/tier4/scenario_simulator_v2/issues/1090>`_ from tier4/refactor/lanelet-id
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into refactor/lanelet-id
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge pull request `#1087 <https://github.com/tier4/scenario_simulator_v2/issues/1087>`_ from tier4/feature/drop_galactic_support
* Replace `std::vector<lanelet::Id>` with `lanelet::Ids`
* Replace `std::int64_t` with `lanelet::Id`
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* remove workbound for galactic
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Michał Kiełczykowski, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1018 <https://github.com/tier4/scenario_simulator_v2/issues/1018>`_ from tier4/fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1022 <https://github.com/tier4/scenario_simulator_v2/issues/1022>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Rename `FollowPolylineTrajectoryParameter` to `PolylineTrajectory`
* Add new message type `traffic_simulator_msgs::msg::PolylineTrajectory`
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Add new message type `traffic_simulator_msgs::msg::Polyline`
* Merge remote-tracking branch 'origin/master' into feat/relative_object_position
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Use CanonicalizedEntityStatus in do_nothing_plugin
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_fe8503' into fix/longitudinal_distance_fixed_master_merged
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, kosuke55, kyoichi-sugahara, yamacir-kit

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* renamed traffic light manager base
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#906 <https://github.com/tier4/scenario_simulator_v2/issues/906>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Update `DoNothingBehavior` to provide accessor for `PollylineTrajectory`
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'master' into feature/rtc_custom_command_action
* Merge pull request `#1011 <https://github.com/tier4/scenario_simulator_v2/issues/1011>`_ from tier4/feature/do_nothing_plugin
* update comment
* modify comment
* add @note comment
* add doNothing()
* sort depends
* fix description
* add do_nothing_plugin package
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit
