^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

22.2.1 (2025-12-25)
-------------------
* Merge branch 'master' into dependabot/uv/markdown-3.10
* Merge branch 'master' into dependabot/uv/markdown-3.10
* Contributors: Kotaro Yoshimoto

22.2.0 (2025-12-25)
-------------------
* Merge branch 'master' into priority
* Merge branch 'master' into priority
* Merge branch 'master' into priority
* Contributors: Kotaro Yoshimoto

22.1.0 (2025-12-25)
-------------------
* Merge branch 'master' into migrate-cppzmq
* Merge branch 'master' into migrate-cppzmq
* Contributors: Daisuke Nishimatsu, Kotaro Yoshimoto

22.0.4 (2025-12-24)
-------------------
* Merge branch 'master' into dependabot/uv/mkdocs-git-revision-date-localized-plugin-1.5.0
* Merge branch 'master' into dependabot/uv/mkdocs-git-revision-date-localized-plugin-1.5.0
* Merge branch 'master' into dependabot/uv/mkdocs-git-revision-date-localized-plugin-1.5.0
* Contributors: Kotaro Yoshimoto

22.0.3 (2025-12-24)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/cache-5
* Merge branch 'master' into dependabot/github_actions/actions/cache-5
* Merge branch 'master' into dependabot/github_actions/actions/cache-5
* Contributors: Kotaro Yoshimoto

22.0.2 (2025-12-24)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/download-artifact-7
* Contributors: Kotaro Yoshimoto

22.0.1 (2025-12-23)
-------------------
* Merge branch 'master' into experiment/signal-testing
* Merge branch 'master' into experiment/signal-testing
* Contributors: ぐるぐる

22.0.0 (2025-12-22)
-------------------

21.3.0 (2025-12-22)
-------------------
* Merge pull request `#1761 <https://github.com/tier4/scenario_simulator_v2/issues/1761>`_ from tier4/fix/RJD-1921-fix-follow-waypoint-controller
* Merge branch 'master' into fix/RJD-1921-fix-follow-waypoint-controller
* Merge branch 'master' into fix/RJD-1921-fix-follow-waypoint-controller
* Merge branch 'master' into fix/RJD-1921-fix-follow-waypoint-controller
* Merge branch 'master' into fix/RJD-1921-fix-follow-waypoint-controller
* Apply clang format
* Change auto&& to const auto& in find_if lambda
* Fix braking typos and improve PredictedEntityStatus code clarity
* Rename state to predicted_status in getPredictedWaypointArrivalState
* Merge branch 'ref/RJD-1921-simplify-follow-trajectory' into fix/RJD-1921-fix-follow-waypoint-controller
* Refactor FollowWaypointController: integrate full entity state and improve acceleration computation
  - replace PredictedState (scalar speed/accel) with PredictedEntityStatus (full EntityStatus wrapper)
  - add PredictedEntityStatus::step() with support for 3D motion simulation and distance_along_lanelet calculation
  - add accelerationWithJerkConstraint() method implementing discrete-time jerk-constrained acceleration with detailed documentation
  - refactor getAccelerationLimits() with improved preconditions documentation and simplified logic using accelerationWithJerkConstraint
  - improve clampAcceleration() edge case handling (backward motion, overspeed) before applying jerk constraints
  - update controller methods (getPredictedWaypointArrivalState, getPredictedStopEntityStatusWithoutConsideringTime, getAcceleration) to accept entity_status and update_entity_status/distance_along_lanelet callbacks
  - implement constrained_brake_velocity lambda in follow_trajectory.cpp using accelerationWithJerkConstraint for smooth final waypoint braking
  - complete final waypoint braking implementation (resolve @todo) with constrained deceleration
  - change remaining_time_to_nearest_timed_waypoint() to return quiet_NaN instead of infinity for consistency
  - add acceptable_overshoot_distance (0.1m) constant and improve error messages in getAccelerationLimits
  - replace isnan checks with isfinite for unified validation
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto

21.2.3 (2025-12-22)
-------------------

21.2.2 (2025-12-19)
-------------------

21.2.1 (2025-12-18)
-------------------
* Revert "Bump version of scenario_simulator_v2 from version 21.2.0 to version 22.0.0"
  This reverts commit 95b2ebee5ed7df8b52a058e92f05e121e2bf75b4.
* Bump version of scenario_simulator_v2 from version 21.2.0 to version 22.0.0
* Contributors: Kotaro Yoshimoto, Release Bot

21.2.0 (2025-12-16)
-------------------
* Merge pull request `#1760 <https://github.com/tier4/scenario_simulator_v2/issues/1760>`_ from tier4/ref/RJD-1921-simplify-follow-trajectory
* Fix clang reformat
* Merge branch 'ref/RJD-1921-simplify-follow-trajectory' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1921-simplify-follow-trajectory
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1921-simplify-follow-trajectory
* Merge branch 'master' into ref/RJD-1921-simplify-follow-trajectory
* Fix clang reformat
* Refactor follow_trajectory - replace trajectory iteration with sliding window model and structured state handlers
  - add initial waypoint insertion in simulator_core.hpp applyFollowTrajectoryAction() (insert current entity pose as vertices[0])
  - expose remaining_distance_tolerance as public constant in follow_waypoint_controller.hpp (change value from 1e-2 to 0.1 for more lenient final waypoint arrival threshold)
  - add geometry/vector3/is_finite.hpp with isfinite() overload for Vector3 types
  - add geometry/vector3/scalar_to_direction_vector.hpp with scalarToDirectionVector() function
  - introduce structured waypoint queries: previous_waypoint(), target_waypoint(), nearest_timed_waypoint() lambda accessors
  - replace linear trajectory iteration with explicit two-waypoint sliding window (vertices[0] = previous, vertices[1] = target)
  - implement comprehensive validation suite: validate_entity_status(), validate_trajectory(), validate_arrival_time(), validate_acceleration_constraints(), validate_desired_motion()
  - refactor distance calculation into composable functions: distance_to_waypoint(), distance_to_target_waypoint(), distance_to_timed_or_final_waypoint()
  - add special case handling for vertices[0] in distance_to_waypoint() (direct entity->previous distance)
  - separate intermediate/final waypoint handling into handle_intermediate_waypoint() and handle_final_waypoint() with explicit state tables
  - add exhaustive documentation tables mapping all entity state/timing combinations to return values (nullopt/EntityStatus/throw/recurse)
  - add structured logging: log_waypoint_action lambda with verbose_action and verbose_move_action flags
  - add verbose_input_state debug logging with entity state, waypoints, distances, and trajectory metadata
  - fix immobile validation: only throw error when entity progressed past previous waypoint but stopped too far from target (allows waiting at trajectory start with target_speed=0.0)
  - replace std::isnan()/std::isinf() with isfinite() for unified validation
  - use 'not' and 'and' operators consistently instead of '!' and '&&'
  - reorganize code into logical sections: WAYPOINT QUERIES, DISTANCE, TIME, VALIDATION, ENTITY UPDATE, VELOCITY, WAYPOINT HANDLING, EXECUTION
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto

21.1.1 (2025-12-10)
-------------------
* Merge branch 'master' into update-sonarqube-settings
* Merge branch 'master' into update-sonarqube-settings
* Contributors: Kotaro Yoshimoto

21.1.0 (2025-12-10)
-------------------
* Merge pull request `#1757 <https://github.com/tier4/scenario_simulator_v2/issues/1757>`_ from tier4/feature/unknown
* Merge branch 'master' into feature/unknown
* Merge branch 'master' into feature/unknown
* Support unknown in TrafficLight::Shape
* Add tests for unknown in TrafficLight::Color
* Support unknown in TrafficLight::Color
* Contributors: Kotaro Yoshimoto

21.0.0 (2025-12-10)
-------------------
* Merge pull request `#1731 <https://github.com/tier4/scenario_simulator_v2/issues/1731>`_ from tier4/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge remote-tracking branch 'tier4/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-2-part1
* Remove unused HdMapUtils::getAltitude member function
* Port the change: ActionNode::getDistanceToTargetEntity -> traffic_simulator::distance::splineDistanceToBoundingBox
* Move implementation of HdMapUtils::getConflictingLaneIds to lanelet_wrapper::lanelet_map::conflictingLaneIds
* Move implementation of HdMapUtils::getConflictingCrosswalkIds to lanelet_wparrer::lanelet_map::conflictingCrosswalkIds
* Port isAltitudeWithinRange function
* Rename isAltitudeMatching -> isAltitudeWithinThreshold
* Move altitude_threshold to header and rename to ALTITUDE_THRESHOLD
* Contributors: Kotaro Yoshimoto, Mateusz Palczuk

20.0.4 (2025-12-09)
-------------------
* Merge pull request `#1764 <https://github.com/tier4/scenario_simulator_v2/issues/1764>`_ from tier4/sort-changelog
* Sort CHANGELOG.rst
* Contributors: Kotaro Yoshimoto

20.0.3 (2025-12-03)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/checkout-6
* Merge branch 'master' into dependabot/github_actions/actions/checkout-6
* Contributors: Kotaro Yoshimoto

20.0.2 (2025-12-03)
-------------------
* Merge branch 'master' into fix/autoware-documentation-link
* Contributors: Kotaro Yoshimoto

20.0.1 (2025-12-03)
-------------------
* Merge branch 'master' into use-maximal-munch
* Contributors: Kotaro Yoshimoto

20.0.0 (2025-12-03)
-------------------
* Merge branch 'master' into fix/double-INF
* Merge remote-tracking branch 'origin/fix/double-INF' into fix/double-INF
* Merge branch 'master' into fix/double-INF
* Contributors: Kotaro Yoshimoto

19.0.3 (2025-11-27)
-------------------

19.0.2 (2025-11-27)
-------------------
* Merge branch 'master' into fix/dependabot-uv
* Merge branch 'master' into fix/dependabot-uv
* Contributors: Kotaro Yoshimoto

19.0.1 (2025-11-27)
-------------------
* Merge branch 'master' into skip-sonarqube-scan-for-dependabots
* Contributors: Kotaro Yoshimoto

19.0.0 (2025-11-27)
-------------------
* Merge pull request `#1478 <https://github.com/tier4/scenario_simulator_v2/issues/1478>`_ from tier4/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Restore fix for negative longitudinal distance in follow trajectory action
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Add comment why include_opposite_direction was changed
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge tag '12.0.2' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Code improvement: avoid unnecessary storage of data in some distance utils functions
* Implement Sonar suggestions
* Change copyright date to 2015
* Add note with the intention
* Revert "Sort before std::unique, so that all duplicates are removed correctly"
  This reverts commit 03ba3cec42cadcdeed31dcd8c726e350127e7a1f.
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'tier4/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance-step-1.5' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Fix typo
* Return const ref to a vector from `CenterPointsCache::centerPoints` and `CenterPointsCache::getCenterPoints`
* Sort before std::unique, so that all duplicates are removed correctly
* Code improvement(lanelet_wrapper::traffic_lights): add const ref to lambda
* Code improvement(lanelet_wrapper::traffic_lights): avoid adding the same traffic light multiple times and performing unnecessary checks
* Code improvement(lanelet_wrapper::traffic_lights): minor improvements
* Code improvement: add missing check in one overload of function `lanelet_wrapper::distance::distanceToTrafficLightStopLine`
  This check is analogical to checks in similar overloaded functions
* Code improvement: avoid unnecessary storage of data and computational overhead in distance to stop line calculation functions
* Code improvement: add const reference wherever applicable and reserve memory in `std::vector`
* Code improvement: change `emplace_back` to `push_back` where emplacing is not needed
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Use vector instead of set when many values are inserted and search is performed only once
* Utilize other overloads and add named constexpr parameters
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'origin/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'origin/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* fix(traffic_simulator): fix after merge
* Merge remote-tracking branch 'origin/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'origin/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge remote-tracking branch 'origin/ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* ref(traffic_simulator): improve hdmaputils::countLaneChanges
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-distance
* ref(clang): revert undesired changes
* ref(traffic_simulator): little lanelet_wrapper refactor
* reftraffic_simulator): use lanelet_wrapper in hdmap_utils, remove separated parts, adapt hdmap_utils tests
* feat(traffic_simulator): adapt test for lanelet_wrapper
* feat(traffic_simulator, behavior_tree_plugin, openscenario_intepreter): use ::distance from lanelet_wrapper instead of hdmap_utils
* feat(traffic_simulator): expand lanelet_wrapper: add ::distance and necessary parts of ::route, ::lanelet_map, ::traffic_lights
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Mateusz Palczuk, Michał Ciasnocha, Tatsuya Yamasaki, robomic

18.5.0 (2025-11-26)
-------------------
* Merge branch 'master' into feature/publisher-noise-imu
* Merge branch 'master' into feature/publisher-noise-imu
* Merge branch 'master' into feature/publisher-noise-imu
* Contributors: Kotaro Yoshimoto

18.4.5 (2025-11-25)
-------------------

18.4.4 (2025-11-21)
-------------------

18.4.3 (2025-11-21)
-------------------
* Merge remote-tracking branch 'origin/master' into fix/poetry-to-uv
  # Conflicts:
  #	poetry.lock
  #	pyproject.toml
* Merge branch 'master' into fix/poetry-to-uv
* Merge branch 'master' into fix/poetry-to-uv
* Merge branch 'master' into fix/poetry-to-uv
* Merge branch 'master' into fix/poetry-to-uv
* Merge branch 'master' into fix/poetry-to-uv
* Merge branch 'master' into fix/poetry-to-uv
* Contributors: Kotaro Yoshimoto, Taiga, ぐるぐる

18.4.2 (2025-11-20)
-------------------
* Merge pull request `#1744 <https://github.com/tier4/scenario_simulator_v2/issues/1744>`_ from tier4/fix/deprecation_error
* Merge branch 'master' into fix/deprecation_error
* Do not treat deprecation as an error
* Contributors: Kotaro Yoshimoto

18.4.1 (2025-11-20)
-------------------

18.4.0 (2025-11-20)
-------------------
* Merge branch 'master' into feature/publisher-noise-pose
* Merge branch 'master' into feature/publisher-noise-pose
* Merge branch 'master' into feature/publisher-noise-pose
* Merge branch 'master' into feature/publisher-noise-pose
* Merge branch 'master' into feature/publisher-noise-pose
* Contributors: Kotaro Yoshimoto

18.3.9 (2025-11-18)
-------------------

18.3.8 (2025-11-18)
-------------------

18.3.7 (2025-11-14)
-------------------
* Merge branch 'master' into dependabot/github_actions/8BitJonny/gh-get-current-pr-4.0.0
* Contributors: Kotaro Yoshimoto

18.3.6 (2025-11-13)
-------------------
* Merge pull request `#1734 <https://github.com/tier4/scenario_simulator_v2/issues/1734>`_ from tier4/fix/multiple_states_on_phase
* Merge branch 'master' into fix/multiple_states_on_phase
* Merge branch 'master' into fix/multiple_states_on_phase
* add TrafficLightsBase::clearTrafficLightsState/addTrafficLightsState
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

18.3.5 (2025-11-13)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/download-artifact-6
* Merge branch 'master' into dependabot/github_actions/actions/download-artifact-6
* Contributors: Taiga

18.3.4 (2025-11-13)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/upload-artifact-5
* Merge branch 'master' into dependabot/github_actions/actions/upload-artifact-5
* Contributors: Taiga

18.3.3 (2025-11-07)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/github-script-8
* Bump version of scenario_simulator_v2 from version 18.3.1 to version 18.3.2
* Merge branch 'master' into dependabot/github_actions/actions/github-script-8
* Merge branch 'master' into dependabot/github_actions/actions/checkout-5
* Merge branch 'master' into dependabot/github_actions/actions/github-script-8
* Merge branch 'master' into dependabot/github_actions/actions/checkout-5
* Contributors: Kotaro Yoshimoto, Taiga

18.3.2 (2025-11-06)
-------------------
* Merge branch 'master' into dependabot/github_actions/actions/checkout-5
* Merge branch 'master' into dependabot/github_actions/actions/checkout-5
* Contributors: Kotaro Yoshimoto, Taiga

18.3.1 (2025-11-04)
-------------------
* Merge branch 'master' into fix/improve-docs
* Merge branch 'master' into fix/improve-docs
* Contributors: Kotaro Yoshimoto, ぐるぐる

18.3.0 (2025-11-04)
-------------------
* Merge branch 'master' into feature/lidar_noise_v1
* Merge branch 'master' into feature/lidar_noise_v1
* Merge branch 'master' into feature/lidar_noise_v1
* Contributors: Kotaro Yoshimoto

18.2.1 (2025-11-04)
-------------------

18.2.0 (2025-11-04)
-------------------
* Merge branch 'master' into feature/publisher-noise-velocity-report
* Contributors: Kotaro Yoshimoto

18.1.1 (2025-10-31)
-------------------

18.1.0 (2025-10-27)
-------------------
* Merge pull request `#1701 <https://github.com/tier4/scenario_simulator_v2/issues/1701>`_ from tier4/feature/trajectory_base_detection_for_pedestrian
  Add Trajectory-Based Pedestrian Front Entity Detection Algorithm
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Merge branch 'master' into feature/trajectory_base_detection_for_pedestrian
* Improve visualization of predicted pedestrian paths.
* Contributors: Kotaro Yoshimoto, Taiga, Taiga Takano

18.0.10 (2025-10-23)
--------------------

18.0.9 (2025-10-23)
-------------------
* Merge branch 'master' into feature/ci-dependabot
* Contributors: Kotaro Yoshimoto

18.0.8 (2025-10-23)
-------------------
* Merge branch 'master' into feature/api-changes-automation
* Merge branch 'master' into feature/api-changes-automation
* Contributors: ぐるぐる

18.0.7 (2025-10-23)
-------------------
* Merge branch 'master' into refactor/raycaster_2
* Merge branch 'master' into refactor/raycaster_2
* Contributors: Kotaro Yoshimoto

18.0.6 (2025-10-22)
-------------------
* Merge branch 'master' into fix/duplicated_nodes
* Contributors: Kotaro Yoshimoto

18.0.5 (2025-10-21)
-------------------
* Merge pull request `#1704 <https://github.com/tier4/scenario_simulator_v2/issues/1704>`_ from tier4/fix/build_error_on_noble
* Merge branch 'master' into fix/build_error_on_noble
* Merge branch 'master' into fix/build_error_on_noble
* Merge branch 'master' into fix/build_error_on_noble
* Add missing header includes
* Contributors: Kotaro Yoshimoto

18.0.4 (2025-10-20)
-------------------
* Merge pull request `#1700 <https://github.com/tier4/scenario_simulator_v2/issues/1700>`_ from mitsudome-r/patch/arm-suppress-warning
* Merge branch 'master' into patch/arm-suppress-warning
* Merge branch 'master' into patch/arm-suppress-warning
* Merge branch 'master' into patch/arm-suppress-warning
* chore: ignore TOLOWER and memaccess in cspell
* fix: suppress warning from Eigen for arm architecture
* Contributors: Kotaro Yoshimoto, Ryohsuke Mitsudome

18.0.3 (2025-10-20)
-------------------

18.0.2 (2025-10-17)
-------------------

18.0.1 (2025-10-16)
-------------------
* Merge branch 'master' into refactor/lidar_sensor
* Contributors: Kotaro Yoshimoto

18.0.0 (2025-10-15)
-------------------
* Merge pull request `#1697 <https://github.com/tier4/scenario_simulator_v2/issues/1697>`_ from tier4/feature/small_fix_new_front_entity_detection_logic
  Fix trajectory-based front entity detection offset handling
* Merge branch 'master' into feature/small_fix_new_front_entity_detection_logic
* Merge branch 'master' into feature/small_fix_new_front_entity_detection_logic
* Filter out invalid trajectory_based_detection_offset\_ values
* Switch path width parameter from override mode to offset mode.
* Contributors: Taiga, Taiga Takano

17.9.1 (2025-10-14)
-------------------
* Merge branch 'master' into fix/RJD-1921-fix-steering-when-autoware-is-overwritten
* Merge branch 'master' into fix/RJD-1921-fix-steering-when-autoware-is-overwritten
* Merge branch 'master' into fix/RJD-1921-fix-steering-when-autoware-is-overwritten
* Contributors: Dawid Moszyński, Kotaro Yoshimoto

17.9.0 (2025-10-14)
-------------------
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Merge branch 'master' into feature/ra
* Remove unuse code
* Add reactive agent plugin implementation and configuration files
* Contributors: Taiga, Taiga Takano

17.8.0 (2025-10-10)
-------------------
* Merge branch 'master' into RJD-1930/see_around
* Merge branch 'master' into RJD-1930/see_around
* Merge branch 'master' into RJD-1930/see_around
* Contributors: Grzegorz Maj, Taiga

17.7.2 (2025-10-08)
-------------------

17.7.1 (2025-10-07)
-------------------

17.7.0 (2025-10-07)
-------------------
* Merge pull request `#1691 <https://github.com/tier4/scenario_simulator_v2/issues/1691>`_ from tier4/feature/whole_new_front_entity_detection_logic
  Whole new front entity detection logic
* Merge branch 'master' into feature/whole_new_front_entity_detection_logic
* Merge branch 'master' into feature/whole_new_front_entity_detection_logic
* Merge branch 'master' into feature/whole_new_front_entity_detection_logic
* Merge branch 'master' into feature/whole_new_front_entity_detection_logic
* Refactor waypoints_marker points assignment for clarity
* fix
* Add trajectory-based front entity detection parameters to VisualizationComponent
* Contributors: Kotaro Yoshimoto, Taiga, Taiga Takano

17.6.3 (2025-10-06)
-------------------
* Merge pull request `#1684 <https://github.com/tier4/scenario_simulator_v2/issues/1684>`_ from tier4/RJD-1921/fta-acceleration-fix
* Merge branch 'master' into RJD-1921/fta-acceleration-fix
* Merge branch 'master' into RJD-1921/fta-acceleration-fix
* Merge branch 'master' into RJD-1921/fta-acceleration-fix
* getTimeRequirementForNonAcceleration step time rounding fix
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Piotr Zyskowski

17.6.2 (2025-10-02)
-------------------
* Merge pull request `#1688 <https://github.com/tier4/scenario_simulator_v2/issues/1688>`_ from tier4/fix/spin-some-deprecated
* Merge branch 'master' into fix/spin-some-deprecated
* Fix TrafficLightsTest error
* Replace spin_some in traffic_simulator test
* Contributors: Kotaro Yoshimoto, f0reachARR

17.6.1 (2025-10-02)
-------------------

17.6.0 (2025-09-29)
-------------------
* Merge branch 'master' into fix/noise_v4
* Contributors: Kotaro Yoshimoto

17.5.3 (2025-09-17)
-------------------
* Merge branch 'master' into fix/macro-xml-validation
* Contributors: Kotaro Yoshimoto

17.5.2 (2025-09-16)
-------------------
* Merge branch 'master' into fix/no-autoware-launch
* Merge branch 'master' into fix/no-autoware-launch
* Contributors: ぐるぐる

17.5.1 (2025-09-11)
-------------------

17.5.0 (2025-09-05)
-------------------
* Merge branch 'master' into fix/approach-stop-and-gap-tolerance
* Contributors: Taiga

17.4.0 (2025-09-04)
-------------------
* Merge branch 'feature/use-yaw-diff-for-front-detection' of github.com:tier4/scenario_simulator_v2 into feature/use-yaw-diff-for-front-detection
* Merge branch 'master' into feature/use-yaw-diff-for-front-detection
* Contributors: Taiga, Taiga Takano

17.3.0 (2025-09-03)
-------------------
* Merge branch 'master' into feature/noise_v4
* Merge branch 'master' into feature/noise_v4
* Merge branch 'master' into feature/noise_v4
* Contributors: Kotaro Yoshimoto

17.2.1 (2025-09-02)
-------------------
* Merge branch 'master' into RJD-1919/fix_health_check
* Merge branch 'master' into RJD-1919/fix_health_check
* Merge branch 'master' into RJD-1919/fix_health_check
* Merge branch 'RJD-1919/fix_health_check' of github.com:tier4/scenario_simulator_v2 into RJD-1919/fix_health_check
* Merge branch 'master' into RJD-1919/fix_health_check
* Contributors: Kotaro Yoshimoto, Michał Ciasnocha, robomic

17.2.0 (2025-09-02)
-------------------
* Merge pull request `#1678 <https://github.com/tier4/scenario_simulator_v2/issues/1678>`_ from tier4/feature/openscenario-per-entity-lateral-collision-margin
  Feature/openscenario per entity lateral collision margin
* refactor: Rename lateralCollisionMargin to lateralCollisionThreshold for consistency
* refactor: Remove unnecessary blank line before clang-format directive
* refactor: Remove unused getLateralCollisionMargin method
* feat: Add newline for improved readability before getLateralCollisionMargin method
* Sort getter/setter macro definitions in alphabetical order
* feat: Update LateralCollisionMargin to use std::optional for improved handling
* Add `lateral_collision_margin`  to vehicle ActionNode
* Contributors: Taiga, Taiga Takano

17.1.4 (2025-08-27)
-------------------

17.1.3 (2025-08-20)
-------------------
* Merge branch 'master' into hotfix/actions-permission
* Contributors: Kotaro Yoshimoto

17.1.2 (2025-08-20)
-------------------
* Merge pull request `#1664 <https://github.com/tier4/scenario_simulator_v2/issues/1664>`_ from xmfcx/refactor/planning-topic-update
* Merge branch 'master' into refactor/planning-topic-update
* Merge branch 'master' into refactor/planning-topic-update
* refactor: change planning output topic name to /planning/trajectory
* Contributors: Kotaro Yoshimoto, M. Fatih Cırıt

17.1.1 (2025-08-20)
-------------------
* Merge branch 'master' into fix/actions-permission
* Merge branch 'master' into fix/actions-permission
* Contributors: ぐるぐる

17.1.0 (2025-08-19)
-------------------
* Merge pull request `#1667 <https://github.com/tier4/scenario_simulator_v2/issues/1667>`_ from tier4/RJD-1907/ss2_vehicle_id_and_localization
* Merge branch 'master' into RJD-1907/ss2_vehicle_id_and_localization
* Merge branch 'master' into RJD-1907/ss2_vehicle_id_and_localization
* Resolve build issue
* Improved code formatting
* RJD-1907 Add support for vehicle_id and initialize_localization
* Contributors: Grzegorz Maj, Kotaro Yoshimoto

17.0.4 (2025-08-18)
-------------------
* Merge branch 'master' into HansRobo-patch-2
* Merge branch 'master' into HansRobo-patch-2
* Merge branch 'master' into HansRobo-patch-2
* Contributors: Kotaro Yoshimoto

17.0.3 (2025-08-18)
-------------------

17.0.2 (2025-08-07)
-------------------

17.0.1 (2025-08-07)
-------------------
* Merge branch 'master' into fix/spinner
* Contributors: Kotaro Yoshimoto

17.0.0 (2025-08-06)
-------------------
* Merge pull request `#1559 <https://github.com/tier4/scenario_simulator_v2/issues/1559>`_ from tier4/refactor/get_loungitudinal_distance
  HdMapUtils refactor `lanelet_wrapper::distance::longitudinalDistance`
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Add DIRTY HACK to make the scenarios pass
* Add more precise distance calculation in FTA
  This also fixes the issue where FTA would crash because of negative distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Update simulation/traffic_simulator/src/utils/pose.cpp
* Return absolute value of distance in distance_along_lanelet in follow_trajectory
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* Merge branch 'master' into refactor/get_loungitudinal_distance
* fix format
* remove `hdmap_utils` from `follow_polyline_trajectory_action`
* remove `hdmap_utils` from `isInLanelet`
* remove `hdmap_utils`from `boundingBoxRelativeLaneletPose`
* remove `hdmap_utils` from `relativeLaneletPose`
* remove `hdmap_utils` from `boundingBoxLaneLongitudinalDistance`
* remove `hdmap_utils` from `longitudinalDistance`
* Add constructors to RoutingConfiguration
* move `getLongitudinalDistance` method
* Contributors: Grzegorz Maj, Kotaro Yoshimoto, Masaya Kataoka, Mateusz Palczuk, Tatsuya Yamasaki, abco20

16.11.0 (2025-08-01)
--------------------
* Merge pull request `#1587 <https://github.com/tier4/scenario_simulator_v2/issues/1587>`_ from tier4/RJD-1769/centerline
* Merge branch 'master' into RJD-1769/centerline
* Delete unused HdMapUtils::overwriteLaneletsCenterline
* Add use_custom_centerline argument and switch behavior
* Merge remote-tracking branch 'origin/master' into RJD-1769/centerline
  # Conflicts:
  #	simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Merge branch 'master' into RJD-1769/centerline
* Merge branch 'master' into RJD-1769/centerline
* RJD-1769 Force creating centerline for all lanelets
* Contributors: Grzegorz Maj, Kotaro Yoshimoto

16.10.0 (2025-07-31)
--------------------
* Merge branch 'master' into feature/traffic_signal_state_v2i
* Contributors: Kotaro Yoshimoto

16.9.0 (2025-07-28)
-------------------
* Merge pull request `#1650 <https://github.com/tier4/scenario_simulator_v2/issues/1650>`_ from tier4/feature/transverse_mercator_projection_support
* Merge branch 'master' into feature/transverse_mercator_projection_support
* Fix: avoid returning reference to temporary YAML::Node, Returning by value is safe and efficient, as `YAML::Node` is a lightweight shared_ptr-like wrapper
* refactor: improve lanelet loader error handling and reduce nesting
* fix: add exception if projector_type is not defined
* Remove unnecessary constexprs
* refactor: enhance lanelet loader with const correctness and reduced allocations
* fix typo
* extracted TM projector producer as a function
* test fix
* clang format
* projector config content error handling
* headers cleanup
* Merge remote-tracking branch 'origin/master' into feature/transverse_mercator_projection_support
* error handling, small cleanup
* transverse mercator support with projection parameters location assumption
* init commit from Yoshimoto-san's patch
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Piotr Zyskowski

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
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Merge branch 'master' into feat/cpp_mock_scenarios_awsim_support
* Contributors: SzymonParapura, Taiga

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
* Isolate protoc output into project named directory
* Add or update xml-model
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
* Merge pull request `#1623 <https://github.com/tier4/scenario_simulator_v2/issues/1623>`_ from tier4/fix/lanelet_pose_in_do_nothing
  Fix/lanelet pose in do nothing
* enable set route lanelet in DoNothingPlugin
* Merge branch 'master' into fix/lanelet_pose_in_do_nothing
* modify lane matching step
* Try lane matching if the output of behavior plugin is not on lane.
* Contributors: Masaya Kataoka, Taiga

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
* Merge pull request `#1597 <https://github.com/tier4/scenario_simulator_v2/issues/1597>`_ from tier4/feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Merge branch 'master' into feature/render-omitted-light-bulb
* Add new line on EOF to lanelet map
* Add infer test
* Remove unnecessary operator
* Merge branch 'master' into feature/render-omitted-light-bulb
* update tests for multiple traffic light
* Add support for render bulbs without state
* Refactor constant
* Activate bulb position inference
* Fix missing override
* Add bulb position inference feature to HdMapUtils
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR, ぐるぐる

16.5.0 (2025-06-03)
-------------------
* Merge pull request `#1611 <https://github.com/tier4/scenario_simulator_v2/issues/1611>`_ from tier4/feature/set_route
* Merge branch 'master' into feature/set_route
* Merge branch 'master' into feature/set_route
* Rename use_lane_level_specification_for_waypoints to use_lane_ids_for_routing
* Use algorithm functions instead of for loop code
* Stop using deprecated version of FieldOperatorApplication::plan in EgoEntity::requestReplanRoute
* Merge branch 'master' into feature/set_route
* Add an exception when failed to get current lanelet in EgoEntity::requestAssignRoute
* Rename FieldOperatorApplication::setRoute/setRoutePoints to plan
* configration => configuration
* use setRoutePoints instead of plan in EgoEntity::requestAssignRoute
* Use route and waypoint appropriately (route = waypoints + goal)
* Interpolate and unique lanelet array for set route API
* Switch by use_lane_level_specification_for_waypoints flag in EgoEntity::requestAssignRoute
* Change redirect target of requestAssignRoute(lanelet_pose[])
* Adjust name of (traffic_simulator::)RouteOption
* Define conversion of RouteOption from v1 to v2
* Add RouteOption v2
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
* check waypoint duplication
* fix spell error
* Merge remote-tracking branch 'origin/master' into feature/speed_up
* add error message
* enable calculate route before requestFollowTrajectory
* enable construct polyline trajectory from spline
* reserve unoderd_map while construct
* Contributors: Masaya Kataoka, Taiga

16.4.3 (2025-05-27)
-------------------
* Merge branch 'master' into refactor/behavior-tree-2
* Bump version of scenario_simulator_v2 from version version 16.4.1 to version 16.4.2
* Merge branch 'master' into refactor/behavior-tree-2
* Contributors: Kotaro Yoshimoto, Taiga

16.4.2 (2025-05-23)
-------------------

16.4.1 (2025-05-23)
-------------------
* Merge pull request `#1602 <https://github.com/tier4/scenario_simulator_v2/issues/1602>`_ from tier4/refactor/behavior-tree-1
  Refactor/behavior_tree-1
* Merge branch 'master' into refactor/behavior-tree-1
* Merge branch 'master' into refactor/behavior-tree-1
* Changed the methods in `behavior_tree.hpp` to use trailing return types.
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

16.4.0 (2025-05-22)
-------------------
* Merge pull request `#1590 <https://github.com/tier4/scenario_simulator_v2/issues/1590>`_ from tier4/feature/change_allow_goal_modification
* Stop using C++ 20 feature to fix a build error
* Merge branch 'master' into feature/change_allow_goal_modification
* Update deprecated notice in EntityBase class
* Add inline namespaces to RouteOption for versioning
* Rename RouteOptions -> RouteOption
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* Adjust deprecation notice from EntityBase::requestAcquirePosition/requestAssignRoute
* Merge branch 'master' into feature/change_allow_goal_modification
* Apply linter
* Add deprecation notice
* Modify MiscObjectEntity implementation to follow EntityBase
* Modify EgoEntity implementation to follow EntityBase
* Modify PedestrianEntity implementation to follow EntityBase
* Modify VehicleEntity implementation to follow EntityBase
* Add EgoEntity::requestAcquirePosition/requestAssignRoute with RouteOptions
* Add traffic_simulator::RouteOptions struct
* Merge branch 'master' into feature/change_allow_goal_modification
* Merge branch 'master' into feature/change_allow_goal_modification
* chore: override allow_goal_modification in EgoEntity::requestAssignRoute only if Controller.Properties.Property.allowGoalModification is defined
* Add allow_goal_modification argument to EgoEntity::requestAcquirePosition / requestAssignRoute
* Add allow_goal_modification argument to FieldOperatorApplication::plan
* Contributors: Kotaro Yoshimoto

16.3.11 (2025-05-21)
--------------------
* Merge pull request `#1579 <https://github.com/tier4/scenario_simulator_v2/issues/1579>`_ from tier4/refactor/lanelet_matching
  [FIX] Lanelet matching with absolute distance
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Merge branch 'master' into refactor/lanelet_matching
* Improve lane matching by using absolute distance for nearest lane selection
* Merge commit '2be47bbd1a1a69ba584d2a37b11b3140e40f5f3d' into refactor/lanelet_matching
* Contributors: Koki Suzuki, Masaya Kataoka, Taiga, koki suzuki

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
* Merge pull request `#1589 <https://github.com/tier4/scenario_simulator_v2/issues/1589>`_ from tier4/fix/mics-objects-model3d
  fix: add missing MiscObjectBehavior when calling spawn api
* Merge branch 'master' into fix/mics-objects-model3d
* Merge branch 'master' into fix/mics-objects-model3d
* fix: add missing MiscObjectBehavior when calling spawn api
* Contributors: Lukasz Chojnacki, Tatsuya Yamasaki

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
* Merge pull request `#1568 <https://github.com/tier4/scenario_simulator_v2/issues/1568>`_ from tier4/RJD-1509/rough_distances
  RJD-1509 Filter nearest NPCs to calculate distance
* Resolve cpp_mock_scenarios failures and clang format
* Review changes
* RJD-1509 Filter nearest NPCs to calculate distance
* Contributors: Grzegorz Maj, Masaya Kataoka

16.3.2 (2025-04-25)
-------------------
* Merge pull request `#1567 <https://github.com/tier4/scenario_simulator_v2/issues/1567>`_ from tier4/RJD-1509/methods_optimization
  RJD-1509 ActionNode methods optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Merge branch 'master' into RJD-1509/methods_optimization
* Review changes
* RJD-1509 Optimize number of distance::longitudinalDistance calls
* Contributors: Grzegorz Maj, Masaya Kataoka

16.3.1 (2025-04-25)
-------------------
* Merge branch 'master' into fix/concealer-7/transition
* Merge branch 'master' into fix/concealer-7/transition
* Merge branch 'master' into fix/concealer-7/transition
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

16.3.0 (2025-04-25)
-------------------
* Merge pull request `#1569 <https://github.com/tier4/scenario_simulator_v2/issues/1569>`_ from tier4/feature/agnocast
* Merge branch 'master' into feature/agnocast
* Merge branch 'master' into feature/agnocast
* Revert "Revert "Merge branch 'master' into feature/agnocast""
  This reverts commit b54960a3492c52964556d54d5943c00cdfa10f50.
* Revert "Merge branch 'master' into feature/agnocast"
  This reverts commit a01992c8e365edd59a52d918cccfec61885234f2, reversing
  changes made to 7f892377f23e4a7bfec460cbfa9f7cdd1b644806.
* Merge branch 'master' into feature/agnocast
* Merge branch 'master' into feature/agnocast
* fix(agnocast): fix unit-test after agnocast support
* Merge branch 'master' into feature/agnocast
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk

16.2.0 (2025-04-24)
-------------------

16.1.4 (2025-04-23)
-------------------
* Merge pull request `#1578 <https://github.com/tier4/scenario_simulator_v2/issues/1578>`_ from tier4/RJD-1752/fix_asserts
  RJD-1752 Fix assert in CanonicalizedEntityStatus constructor
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* Merge branch 'master' into RJD-1752/fix_asserts
* RJD-1752 Fix wrongly placed assert in CanonicalizedEntityStatus constructor
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
* Fix errors
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
* Fix copyright notice date
* Change runtime check to compiletime check
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Adjust `EntityBase::as` to return reference instead of shared_ptr
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Fix typo
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Use EntityBase `is` and `as` functions
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Use modern initialization
* Remove unused member
* Refactor: configuration -> configuration\_
* Refactor: Use in class initializer
* Refactor: Replace std::is_same<>::value with std::is_same_v<>
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* ref(traffic_simulator): apply sonarcloud requested changes
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge remote-tracking branch 'origin/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-refactor
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-with-middle
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-with-middle
* Fix Ego pose setting
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base' into RJD-1057-remove-functions-forwarded-to-entity-base
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* fix(api): fix init order
* ref(entity_manager): refactor the entire Entity_manager, separate sections
* ref(entity_manager, entity_base): add as() to EntityBase, remove unnecessary from EntityManager
* ref(api): refactor the entire API, remove forwarding macro, separate sections
* ref(api, entity_manager): improve spawn, spawnEntity - return shared_ptr
* feat(traffic_simulator): separate ostream helpers, extend it
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
* Merge branch 'master' into fix/scenario_name
* Merge branch 'master' into fix/scenario_name
* Contributors: Masaya Kataoka

15.0.6 (2025-04-09)
-------------------
* fix: move workaround
* fix: add workaround to avoid build CI fail due to tinyxml2
* Contributors: Kotaro Yoshimoto

15.0.5 (2025-04-04)
-------------------
* Merge branch 'master' into fix-turn-indicator-report
* Merge branch 'master' into fix-turn-indicator-report
* Contributors: Kem (TiankuiXian), Kotaro Yoshimoto

15.0.4 (2025-04-03)
-------------------

15.0.3 (2025-04-03)
-------------------
* Merge pull request `#1555 <https://github.com/tier4/scenario_simulator_v2/issues/1555>`_ from tier4/feature/remove-trajectory-subscription
  Remove Autoware trajectory subscription
* Merge branch 'master' into feature/remove-trajectory-subscription
* Fix spelling
* Merge remote-tracking branch 'tier4/master' into feature/remove-trajectory-subscription
* Remove Autoware trajectory subscription
* Contributors: Mateusz Palczuk, Tatsuya Yamasaki

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
* remove hdmap_utils from `boundingBoxLaneLateralDistance`
* remove hdmap_utils from `lateralDistance` function
* move `getLateralDistance` method
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

14.2.3 (2025-03-28)
-------------------
* Merge pull request `#1549 <https://github.com/tier4/scenario_simulator_v2/issues/1549>`_ from tier4/refactor/concealer-7
  Refactor/concealer 7
* Merge branch 'master' into refactor/concealer-7
* Merge branch 'master' into refactor/concealer-7
* Merge branch 'master' into refactor/concealer-7
* Merge remote-tracking branch 'origin/master' into refactor/concealer-7
* Add new struct `LegacyAutowareState`
* Merge remote-tracking branch 'origin/master' into refactor/concealer-7
* Remove data member `concealer::FieldOperatorApplication::autoware_state`
* Contributors: Tatsuya Yamasaki, yamacir-kit

14.2.2 (2025-03-27)
-------------------
* Merge pull request `#1543 <https://github.com/tier4/scenario_simulator_v2/issues/1543>`_ from tier4/RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'master' into RJD-1057/unify-spawn
* Merge branch 'RJD1057/change-order-of-members' into RJD-1057/unify-spawn
* Apply formatter
* Unify spawn API
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR, ぐるぐる

14.2.1 (2025-03-27)
-------------------
* Merge pull request `#1552 <https://github.com/tier4/scenario_simulator_v2/issues/1552>`_ from tier4/RJD-1057/no-specific-param-in-manager
  RJD-1057: Remove some entity type specific functions in EntityManager
* Merge remote-tracking branch 'origin/master' into RJD-1057/no-specific-param-in-manager
* Use as<> for get parameters of each type of entity
* Add `as` to EntityBase for type cast
* Contributors: Masaya Kataoka, f0reachARR

14.2.0 (2025-03-26)
-------------------
* Merge pull request `#1550 <https://github.com/tier4/scenario_simulator_v2/issues/1550>`_ from tier4/refactor/lanelet_wrapper_traffic_lights
  HdMapUtils refactor `lanelet_wrapper::traffic_lights`
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Merge branch 'master' into refactor/lanelet_wrapper_traffic_lights
* Add traffic lights functions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

14.1.0 (2025-03-25)
-------------------
* Merge pull request `#1553 <https://github.com/tier4/scenario_simulator_v2/issues/1553>`_ from tier4/feature/revival_getStopLineIds
  Renamed getStopLineIds to stopLineIds to revive functionality.
* Added unit tests for the `stopLineIds` function.
* Merge branch 'master' into feature/revival_getStopLineIds
* Renamed getStopLineIds to stopLineIds to revive functionality.
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

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
* Merge pull request `#1540 <https://github.com/tier4/scenario_simulator_v2/issues/1540>`_ from tier4/RJD1057/change-order-of-members
  RJD-1057: Change order of functions / variables
* Merge remote-tracking branch 'origin/master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Merge branch 'master' into RJD1057/change-order-of-members
* Split constructor to some methods
* Remove unused method
* Change member declaration for styling
* Remove unused include
* Add missing const
* Remove unnecessary namespace
* Add comment for method category
* Remove FORWARD_TO_ENTITY_MANAGER
* Rename configuration like other private variables
* Add Type suffix for some typename
* Use auto for method
* Refactor function order in traffic_simulator
* Refactor order of api
* Refactor function order in EntityManager
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, f0reachARR, ぐるぐる

14.0.1 (2025-03-18)
-------------------
* Merge pull request `#1536 <https://github.com/tier4/scenario_simulator_v2/issues/1536>`_ from tier4/feature/support-internal-repository
  Make Certain Member Functions Public for context_gamma_planner
* Merge branch 'master' into feature/support-internal-repository
* Merge branch 'master' into feature/support-internal-repository
* Merge branch 'master' into feature/support-internal-repository
* Fix for intternal repository
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

14.0.0 (2025-03-14)
-------------------
* Merge pull request `#1546 <https://github.com/tier4/scenario_simulator_v2/issues/1546>`_ from tier4/refactor/concealer-5
  Refactor/concealer 5
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Cleanup
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Restore member function template `API::getROS2Parameter`
* Merge remote-tracking branch 'origin/master' into refactor/concealer-5
* Replace miscellaneous `get_parameter` calls with `common::getParameter`
* Remove member function `API::getROS2Parameter`
* Remove free function `traffic_simulator::getParameter`
* Contributors: Masaya Kataoka, yamacir-kit

13.0.0 (2025-03-14)
-------------------
* Merge pull request `#1538 <https://github.com/tier4/scenario_simulator_v2/issues/1538>`_ from tier4/refactor/lanelet_wrapper_distance_to_stop_line
  HdMapUtils refactor `lanelet_wrapper::distance::distanceToStopLine`
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Merge branch 'master' into refactor/lanelet_wrapper_distance_to_stop_line
* Fix include order
* move `getDistanceToStopLine` method
* move `getStopLinePolygon` method
* move `getStopLineIds` method and `getStopLineIdsOnPath` method
* add `trafficSignsOnPath`  method
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

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
* Merge pull request `#1548 <https://github.com/tier4/scenario_simulator_v2/issues/1548>`_ from tier4/chore/update_rviz_config_for_motion_veloicty_planner
  Add missing debug marker for motion velocity planner
* add debug marker for motion velocity obstacle\_(stop|slow_down|cruise) module
* Contributors: Autumn60, Kotaro Yoshimoto

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
* Merge pull request `#1534 <https://github.com/tier4/scenario_simulator_v2/issues/1534>`_ from tier4/refactor/concealer-4
  Refactor/concealer 4
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Merge branch 'master' into refactor/concealer-4
* Reformat
* Remove macro `DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE`
* Remove member function `FieldOperatorApplication::rethrow`
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki, yamacir-kit

12.2.0 (2025-03-10)
-------------------
* Merge branch 'master' into feat/add_new_vehicle_model
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
* Merge pull request `#1539 <https://github.com/tier4/scenario_simulator_v2/issues/1539>`_ from tier4/RJD-1057/reorgnize-ostream-helper
  RJD-1057: Reorganize ostream operators
* fix: add missing ")" and structured output
* Merge branch 'master' into RJD-1057/reorgnize-ostream-helper
* Update ostream helper
  Co-authored-by: Dawid Moszynski <dawid.moszynski@robotec.ai>
  Co-authored-by: Mateusz Palczuk <mateusz.palczuk@robotec.ai>
* Contributors: Kotaro Yoshimoto, f0reachARR, ぐるぐる

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
* remove hdmap_utils from `distanceToLeftLaneBound` and `distanceToRightLaneBound`
* move `getLeftBound` and `getRightBound` methods
* Add toPolygon function
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
* Merge pull request `#1531 <https://github.com/tier4/scenario_simulator_v2/issues/1531>`_ from tier4/refactor/lanelet_wrapper_route
  HdMapUtils refactor lanelet_wrapper::route
* Merge branch 'master' into refactor/lanelet_wrapper_route
* fix format
* use non member route function `getAlternativeLaneletPoseBaseOnShortestRouteFrom`
* use non member route function `findRoutableAlternativeLaneletPoseFrom`
* add route::route utils
* Implement static route function in lanelet_wrapper
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, abco20

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
* Merge pull request `#1530 <https://github.com/tier4/scenario_simulator_v2/issues/1530>`_ from tier4/fix/visualization_triangle
  fix: remove double applied orientation for visualization entity triangle marker
* fix: remove double applied orientation for visualization entity triangle marker
* Contributors: Kotaro Yoshimoto

10.3.1 (2025-02-17)
-------------------
* Merge branch 'master' into fix/use-capital-as
* Contributors: Kotaro Yoshimoto

10.3.0 (2025-02-14)
-------------------
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
* Contributors: Tatsuya Yamasaki, yamacir-kit

10.2.0 (2025-02-14)
-------------------
* Merge branch 'master' into feature/rosbag_storage
* Merge branch 'master' into feature/rosbag_storage
* Merge branch 'master' into feature/rosbag_storage
* Contributors: Kotaro Yoshimoto

10.1.2 (2025-02-14)
-------------------
* Merge pull request `#1527 <https://github.com/tier4/scenario_simulator_v2/issues/1527>`_ from tier4/fix/copyright_date
  Change copyright date to 2015
* Change copyright date to 2015
* Contributors: Tatsuya Yamasaki, abco20

10.1.1 (2025-02-13)
-------------------
* Merge branch 'master' into doc/comment_about_respawn_ego
* Contributors: Masaya Kataoka

10.1.0 (2025-02-12)
-------------------
* Merge branch 'master' into fix/respawn-ego-test
* Contributors: Kotaro Yoshimoto

10.0.0 (2025-02-07)
-------------------
* Merge pull request `#1500 <https://github.com/tier4/scenario_simulator_v2/issues/1500>`_ from tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
  [extra] RJD-1057 (3+/5): change getEntity return type to the reference
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge remote-tracking branch 'tier4/RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* Merge branch 'RJD-1057-remove-functions-forwarded-to-entity-base-middle' into RJD-1057-remove-functions-forwarded-to-entity-base-middle-get-entity
* ref(traffic_simulator, simulator_core): apply solar required changes
* ref(traffic_simulator): apply clang reformat
* ref(traffic_simulator): add guard to EntityBase - remove copy and move constuctor, as well as assignments
* ref(traffic_simulator, simulator_core): rename getEntityOrNullptr to getEntityPointer
* ref(traffic_simulator): change getEntity return type to reference, adapt api and core
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
* Merge pull request `#1493 <https://github.com/tier4/scenario_simulator_v2/issues/1493>`_ from tier4/fix/slope_inaccuracies
  Fix/slope inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* fix(traffic_simulator): Fix an issue with lanelet transition
* fix(traffic_simulator) Fix an issue with follow trajectory action
  - Updated makeUpdatedStatus to fix an issue with toCanonicalizedLaneletPose
* Merge branch 'master' into fix/slope_inaccuracies
* ref(traffic_simulator): Refactor comments
* ref(traffic_simulator): Refactor comments
* ref(traffic_simulator): Refactor comments
* ref(traffic_simulator> Refactor code according to the lastest master branch changes
* Merge branch 'master' into fix/slope_inaccuracies
* fix(traffic_simulator) Improve lanelet transition handling for entity position updates
* ref(traffic_simulator) Refactor updateEntityPositionForLaneletTransition
* Merge branch 'master' into fix/slope_inaccuracies
* fix(traffic_simulator): Fix lanelet slope inaccuracies
  - Adjusted entity position when transitioning between lanelets to reduce slope errors
* Merge branch 'master' into fix/slope_inaccuracies
* Merge branch 'master' into fix/slope_inaccuracies
* fix(traffic_simulator): revert adjustOrientationAndOzPosition  change (lanelet rpy)
* ref(traffic_simulator): tidy up after moveTowardsLaneletPose development
* fix(behavior_tree_plugin, traffic_simulator): fix moveTowardsLaneletPose for WalkStraightAction, tidy up
* tmp(behavior_tree_plugin, traffic_simulator): moveTowardsLaneletPose debug
* Merge branch 'master' into fix/slope_inaccuracies
* refactor(traffic_simulator) fix spell check issue
* fix(traffic_simulator) Fix an issue with negative longitudinal displacement
* fix(traffis_simulator): use next canonicalized lanelet pose in pose::moveTowardsLaneletPose
* ref(behavior_tree_plugin, traffic_simulator): improve comments
* fix(traffic_simulator): fix moveTowardsLaneletPose
* ref(traffic_simulator): remove irrelevant toMapPosition
* feat(traffic_simulator): improve moveTowardsLaneletPose to calc LaneletPose and return it
* ref(traffic_simulator): cleanup after removal of pose::moveAlongLanelet
* fix(behavior_tree_plugin): use moveToLaneletPose in calculateUpdatedEntityStatusInWorldFrame
* fix(traffic_simulator): fix moveToLaneletPose in FollowTrajectoryAction
* ref(traffic_simulator): fix pose::moveToTargetLaneletPose
* ref(traffic_simulator): separete pose::moveToTargetPosition
* ref(behavior_tree_plugin, traffic_simulator): separate pose::moveAlongLanelet
* fix(behavior_tree_plugin)  Fix lanelet slope inaccuracies - walk straight action
* fix(traffic_simulator) Fix lanelet slope inaccuracies - follow trajectory action
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, SzymonParapura

9.1.0 (2025-02-04)
------------------
* Merge pull request `#1487 <https://github.com/tier4/scenario_simulator_v2/issues/1487>`_ from tier4/RJD-1489/NpcCenterLine
* Remove std::midpoint
* Review changes
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* SonarCloud issues changes part 2
* SonarCloud issues changes
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into RJD-1489/NpcCenterLine
* Formatting changes
* Review changes
* Move tranformToRoutableCanonicalizedLaneletPose to pose namespace
* Move distance calculation method to utils
* Review changes
* Add new algorithm
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Merge branch 'master' into RJD-1489/NpcCenterLine
* Contributors: Dawid Moszyński, Grzegorz Maj, Kotaro Yoshimoto

9.0.3 (2025-01-31)
------------------
* Merge pull request `#1509 <https://github.com/tier4/scenario_simulator_v2/issues/1509>`_ from tier4/RJD-1505/fix_slope_acceleration_sign
* Merge branch 'master' into RJD-1505/fix_slope_acceleration_sign
* chore: fix a mistake during solving the merge conflict
* Merge branch 'master' into RJD-1505/fix_slope_acceleration_sign
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into RJD-1505/fix_slope_acceleration_sign
* Remove noexcept
* Change sign of slope acceleration  and check canonicalization status in acceleration calculation
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
* remove double canonicalization
* merge 8.0.2
* remove recursive calls after merge
* Merge tag '7.4.7' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Revert "move functions from EgoEntity to FieldOperatorApplication"
  This reverts commit 8d90c0e2c2f66795f299859bfa675ef2f11abdd3.
* Revert "remove function shadowing"
  This reverts commit 82d6085784271ea59d8fa69f359e0437543e9d57.
* remove function shadowing
* move functions from EgoEntity to FieldOperatorApplication
* Merge remote-tracking branch 'origin/master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(traffic_simulator, simulator_interface): revert format unexpected changes
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
* ref(traffic_simulator): tiny formatting change
* ref(traffic_simulator): improve requestClearRoute call for ego_entity
* Merge branch 'master' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(traffic_simulator): add const to LongitudinalSpeedPlanner::isTargetSpeedReached arguments
* Merge remote-tracking branch 'origin/RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* ref(traffic_simulator): improve requestLaneChange, getEgoName
* ref(traffic_simulator, simulator_core): improve overall
* ref(cpp_mock_scenarios): improve overall
* ref(behavior_tree,traffic_simulator): improve code by basing on sonarcloud
* Merge remote-tracking branch 'origin/RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge branch 'RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'tier4/RJD-1057-traffic-lights-tests' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* Merge remote-tracking branch 'tier4/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base-middle
* feat(entity_base, entity_status): improve ::setStatus
* Fix concealer local address returning
* feat(simulator_core, api, entity_base, cpp_mock): move setEntityStatus to EntityBase, remove from api
* feat(api, simulator_core): emove getTimeHeadway from API, use directly in calc it directly in evaluateTimeHeadway
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-remove-functions-forwarded-to-entity-base
* feat(ego_entity, sumulator_core): remove asFieldOperatorApplication, develop getEgoEntity and dedicated methods in EgoEntity
* feat(api, entity_manager, cpp_mock): rename entityExist to isEntitySpawned, move checkCollision directly to API
* feat(entity_manager, simulator_core): remove activateOutOfRangeJob forwarding
* feat(entity_manager, behavior_tree, cpp_mock): remove getCurrentAction forwarding, set "waiting" as init action state in behavior_tree
* feat(entity_base, traffic_simulator, simulator_core): remove forwarding request*, move requestLaneChange to EntityBase
* feat(entity_base, traffic_simulator, simulator_core): remove forwarding setBehaviorParameter and setVelocityLimit
* feat(entity_base, traffic_simulator): move isStopping to EntityBase, develop templated is() in EntityBase and use it, rename isEgoSpawned to isAnyEgoSpawned, refactor
* feat(entity_base, traffic_simulator): remove forwarding setters to EntityBase, also some getters, left setVelocityLimit and setBehaviorParameter
* feat(entity_base, traffic_simulator): rename laneMatchingSucceed to isInLanelet, remove forwarding
* feat(entity_base, traffic_simulator): move reachPosition as isInPosition to EntityBase, remove forwarding
* feat(entity_base, traffic_simulator, cpp_mock): move isInLanelet to EntityBase, remove forwarding
* Merge remote-tracking branch 'origin/RJD-1056-remove-npc-logic-started' into RJD-1057-remove-functions-forwarded-to-entity-base
* Merge remote-tracking branch 'origin/RJD-1056-remove-current-time-step-time' into RJD-1057-remove-functions-forwarded-to-entity-base
* feat(cpp_mock, traffic_simulator): change getEntity, use getEntityOrNullptr
* Remove forwarding of getStandStillDuration in EntityManager and API
* Remove forwarding of get2DPolygon in EntityManager
* Remove forwarding of getBoundingBox in EntityManager and API
* Remove forwarding of getLinearJerk in EntityManager and API
* Remove forwarding of getCurrentAccel in EntityManager and API
* Remove forwarding of getCurrentTwist in EntityManager and API
* Remove forwarding of getBehaviorParameter in EntityManager and API
* Remove forwarding of getEntityStatusBeforeUpdate in EntityManager and API
* Add const
* Remove forwarding of getEntityStatus in EntityManager and API
* Forward getEntityOrThrow from API to EntityManager
* Remove forwarding of getEntityType in EntityManager
* Add getEntityOrThrow function to mimic forwarding functionality where exceptions were thrown
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
* Add reader functions to cache classes to mitigate direct data members access and having to lock the mutex in different places
* Revert "Refactor lanelet_wrapper cache classes: make top level public member functions acquire resources and make non-public member functions NOT thread safe"
  This reverts commit 41f9dd1e226ea827444aed1d01c207d10b9b61fa.
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Remove unnecessary comments
* Adjust comments to use "/// @note" convention
* Refactor lanelet_wrapper cache classes: make top level public member functions acquire resources and make non-public member functions NOT thread safe
  This change is a proposal of improving thread safety - now the whole bodies of public member functions can be executed at once without the risk of resources being modified in the middle of the function execution
* Clean cache code - move cache classes member variables to the end of class declaration
* Move lanelet_wrapper cache classes member values and some functions to private
* Add comment explaining parameters
* Rename variable to match function name change
* Rename `borderlinePoses` -> `noNextLaneletPoses`
* Add comment explaining what `lanelet_map::borderlinePoses` does
* Perform calculations at compile time in lanelet_wrapper::pose::toLaneletPose
* Fix lanelet pose matching calculation bug
* fix(lanelet_wrapper): fix toLaneletPose
* doc(lanelet_wrapper): improve lanelet_wrappe::pose::canonicalizeLaneletPose description
* fix(lanelet_wrapper): fix pose toLaneletPose typo ";"
* fix(lanelet_wrapper): fix typo centerline
* Merge branch 'ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose' of github.com:tier4/scenario_simulator_v2 into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(lanelet_wrapper): add toLaneletPose yaw_threshold description
* ref(traffic_simulator): add comments to lanelet_wrapper::pose, reduce the number of intermediate variables
* Merge remote-tracking branch 'origin/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Update simulation/traffic_simulator/src/traffic/traffic_controller.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_map.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_wrapper.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/lanelet_loader.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/utils/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/utils/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/lanelet_wrapper/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/src/utils/pose.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Update simulation/traffic_simulator/include/traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(traffic_simulator): apply solar changes
* ref(traffic_simulator): apply sonar required changes
* Merge remote-tracking branch 'origin' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(traffic_simulator): refactor laneletAltitude and isAltitudeMatching after merge
* Merge remote-tracking branch 'origin' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* fix(traffic_simulator): fix follow_trajectory_action issue, add orientation to distance calc, remove toCanonicalizedLaneletPose(point...) because it can cause a another issues
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(traffic_simulator): improve hdmaputils::countLaneChanges
* fix(traffic_simulator): adapt lanelet-wapper::pose to changes in hdmap_utils
* Merge branch 'master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* fix(traffic_simulator): fix lanelet_wrapper::pose::alternativeLaneletPoses
* fix(traffic_simulator, simple_sensor_simulator): fix after merge, apply sonar recommendations
* Merge remote-tracking branch 'origin/master' into ref/RJD-1387-hdmap-utils-to-lanelet-wrapper-pose
* ref(traffic_simulator): apply sonar recommendations to lanelet_wrapper and utils
* feat(traffic_simulator): separate lanelet_loader, provide const TrafficRulesWithRoutingGraph
* ref(traffic_simulator): improve Configuration, traffic_rules, lanelet_wrapper
* fix(traffic_simulator): fix hdmap_utils tests
* fix(traffic_simulator): add EOF to traffic_rules
* fix(traffic_simmulator): fix spell check
* feat(traffic_simulator): use lanelet_wrapper::lanelet_map in the rest of hdmap_utils, adapt hdmap_utils test to lanelet_wrapper::lanelet_map
* feat(traffic_simulator, behavior_tree_plugin): use lanelet_wrapper::lanelet_map in parts previusly overlooked
* feat(traffic_simulator): use lanelet_wrapper::pose in the rest of hdmap_utils, adapt hdmap_utils test to lanelet_wrapper::pose
* feat(traffic_simulator, behavior_tree_plugin): use lanelet_wrapper::pose in parts previously overlooked
* ref(traffic_simulator): remove unused parts lanelet_wrapper
* feat(traffic_simulator, random_test_runner): adapt tests for using pose:: from lanelet_wrapper
* feat(traffic_simulator, openscenario_interpreter, behavior_tree_plugin, simple_sensor_simulator): use pose:: from lanelet_wrapper instead of hdmap_utils methods
* feat(traffic_simulator): add lanelet_wrapper as a replacement for hdmap_utils, at this point mainly for pose calculations
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
* Merge pull request `#1502 <https://github.com/tier4/scenario_simulator_v2/issues/1502>`_ from tier4/fix/pass_despawn_function_in_constructor
  add despawn_function argument
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* Merge remote-tracking branch 'origin/master' into fix/pass_despawn_function_in_constructor
* rename despawn_function to despawn
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* Merge branch 'master' into fix/pass_despawn_function_in_constructor
* add despawn_function argument
* Contributors: Masaya Kataoka

7.4.4 (2025-01-09)
------------------
* Merge pull request `#1497 <https://github.com/tier4/scenario_simulator_v2/issues/1497>`_ from tier4/refactor/concealer-2
  Refactor/concealer 2
* Merge branch 'master' into refactor/concealer-2
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Remove some member accessor from struct `FieldOperatorApplication`
* Remove member function `FieldOperatorApplication::getAutowareStateName`
* Cleanup `ControlModeCommand` service callback
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Merge remote-tracking branch 'origin/master' into refactor/concealer-2
* Update `EgoEntity` to have `FieldOperatorApplication` as its base class
* Update `EgoEntity` to have `FieldOperatorApplication` as non-pointer
* Remove static member function `EgoEntity::makeFieldOperatorApplication`
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.4.3 (2025-01-07)
------------------
* Merge pull request `#1499 <https://github.com/tier4/scenario_simulator_v2/issues/1499>`_ from tier4/tmp/pc-patch
* Merge branch 'master' into tmp/pc-patch
* fix: implementation
* fix: implementation
* perf(traffic_light): add condition to add new traffic light
* Contributors: Kotaro Yoshimoto, satoshi-ota

7.4.2 (2025-01-07)
------------------
* Merge pull request `#1501 <https://github.com/tier4/scenario_simulator_v2/issues/1501>`_ from tier4/feature/new_rviz
* chore: update rviz config based on autoware.rviz
* Contributors: Kotaro Yoshimoto

7.4.1 (2024-12-24)
------------------
* Merge pull request `#1498 <https://github.com/tier4/scenario_simulator_v2/issues/1498>`_ from tier4/fix/canonicalize_function
  Fix/canonicalize function
* Merge remote-tracking branch 'origin/master' into fix/canonicalize_function
* Merge remote-tracking branch 'origin/master' into fix/canonicalize_function
* Fix source code
* Test cases were added, and the source code was reverted to its pre-modified state so that the added test cases would fail.
* apply reformat
* catch common::SemanticError inside traffic_simulator::pose::canonicalize function
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
* remove unnecessary include directives
* remove unnecessary funcs
* code style
* code style
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* simplify auto_sink logic
* TrafficSinkConfig
* AutoSinkConfig
* review suggestions
* remove lanelet_id from the constructor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* reorder fix
* Merge branch 'master' into RJD-1457/traffic_sink_refactor
* spell-check happy
* TrafficSink refactor with despawn functionality
* Merge tag '6.0.1' into RJD-1457/traffic_sink_refactor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/enable_specify_entity_type_in_autosink
* Merge branch 'master' into feature/enable_specify_entity_type_in_autosink
* remove warning
* remove sonarcloud warning
* apply reformat
* use uint8_t instead of traffic_simulator_msgs::msg::EntityType
* fix typo
* fix typo
* fix typo
* enable set traffic sink in cpp scenario
* enable compare entity type
* add doxygen comment
* add sinkable_entity_type to the configuration class
* add sinkable_entity_type variable
* add get_entity_type function
* rename variable in TrafficController class
* rename argument and member variables
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

7.3.5 (2024-12-20)
------------------
* Merge pull request `#1488 <https://github.com/tier4/scenario_simulator_v2/issues/1488>`_ from tier4/refactor/concealer-1
  Refactor/concealer 1
* Merge branch 'master' into refactor/concealer-1
* Merge branch 'master' into refactor/concealer-1
* Merge branch 'master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Remove class template `FieldOperatorApplicationFor<T>`
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Merge remote-tracking branch 'origin/master' into refactor/concealer-1
* Remove member function `FieldOperatorApplication::restrictTargetSpeed`
* Remove class `concealer::Autoware`
* Contributors: Tatsuya Yamasaki, yamacir-kit

7.3.4 (2024-12-20)
------------------
* Merge pull request `#1490 <https://github.com/tier4/scenario_simulator_v2/issues/1490>`_ from tier4/feature/is_in_intersection
  Feature/is in intersection
* Merge branch 'master' into feature/is_in_intersection
* Merge remote-tracking branch 'origin/master' into feature/is_in_intersection
* add test case for isInIntersection function
* Merge remote-tracking branch 'origin/master' into feature/is_in_intersection
* add auto HdMapUtils::isInIntersection(const lanelet::Id lanelet_id) const -> bool function
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
* Merge pull request `#1481 <https://github.com/tier4/scenario_simulator_v2/issues/1481>`_ from tier4/feature/multi-level-lanelet-support
  Feature/multi level lanelet support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* Merge branch 'master' into feature/multi-level-lanelet-support
* fix(traffic_simulator): revert clang changes
* ref(traffic_simulator, simple_sensor_simulator): refactor altitude checks
* Merge branch 'master' into feature/multi-level-lanelet-support
* [RJD-1370] Fix 3D Lanelet Matching Issue in cpp_mock_scenario
  - Updated the makeRandomPose method to correctly support 3D lanelet matching.
* Fix an issue with unit tests - distanceTest
* Merge branch 'master' into feature/multi-level-lanelet-support
* Removed unrecognized words because spell-check flagged them as invalid
* [RJD-1369] Improve lanelet matching - 3D support
  - Enhanced lanelet matching algorithm (`toLaneletPose` method) by incorporating lanelet altitude.
  - Defined the `altitude_threshold` parameter that sets the maximum altitude difference to determine when an entity can be matched with a specific lanelet.
* Contributors: Dawid Moszynski, Kotaro Yoshimoto, SzymonParapura

7.2.0 (2024-12-16)
------------------
* Merge pull request `#1484 <https://github.com/tier4/scenario_simulator_v2/issues/1484>`_ from tier4/RJD-736/autoware_msgs_support_and_localization_sim_mode_support
  Rjd 736/autoware msgs support and localization sim mode support
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
* Add launch argument `simulate_localization` to `scenario_test_runner`
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
* Merge pull request `#1465 <https://github.com/tier4/scenario_simulator_v2/issues/1465>`_ from tier4/fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* Merge branch 'master' into fix/request-enable-autoware-control
* fix(api): request enable autoware control
* Contributors: Kotaro Yoshimoto, satoshi-ota

7.0.2 (2024-12-12)
------------------
* Merge pull request `#1470 <https://github.com/tier4/scenario_simulator_v2/issues/1470>`_ from tier4/fix/snor-cloud-issue-8-1
  Fix/sonor cloud issue 8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Merge branch 'master' into fix/snor-cloud-issue-8-1
* Remove the commented out code.
  Replace this declaration by a structured binding declaration.
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

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
* chore: apply linter
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* fix: replace "awf/universe" with "awf/universe/20240605" for architecture_type
* chore: delete a test using autoware_auto_perception_msgs
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
  # Conflicts:
  #	simulation/traffic_simulator/src/traffic_lights/traffic_light_publisher.cpp
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* refactor: added handling when autoware_perception_msgs/msg/traffic_signal_array.hpp is not present
* chore: delete wrongly doubled word
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
* fix: delete autoware_auto_xx_msgs from include
* Merge branch 'master' into RJD-736/autoware_msgs_support
* apply linter
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
* Fix conflicts in includes
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Revert "add autoware\_ prefix"
  This reverts commit a3591f038b30ca35f9188d552556a482a2ffbe17.
* add autoware\_ prefix
* Merge remote-tracking branch 'origin/feature/manual_on_follow_trajectory' into feature/manual_on_follow_trajectory_not_auto
  # Conflicts:
  #	simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge pull request `#1277 <https://github.com/tier4/scenario_simulator_v2/issues/1277>`_ from tier4/RJD-736/autoware_msgs_support_new_traffic_light_group
  Rjd 736/autoware msgs support new traffic light group
* Merge remote-tracking branch 'origin/RJD-736/autoware_msgs_support_new_traffic_light_group' into feature/manual_on_follow_trajectory_not_auto
  # Conflicts:
  #	external/concealer/src/autoware_universe.cpp
* feat(traffic_simulator): support TrafficLightGroup for V2I traffic lights
* feat(traffic_simulator): support TrafficLightGroupArray in TrafficLightPublisher
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Merge remote-tracking branch 'origin/master' into RJD-736/autoware_msgs_support
* Delete unused autoware_auto_system_msgs package
* Port DetectedObjects and TrackedObjects
* Port TrafficSignalArray from autoware_auto_perception_msgs to autoware_perception_msgs
* Port AckermannControlCommand to Control
* Port HADMapBin to LaneletMapBin
* Port GearCommand
* Delete unused vehicle_state_command and vehicle_control_command
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Michał Kiełczykowski, Yutaka Kondo

6.3.1 (2024-12-10)
------------------
* Merge branch 'master' into fix/sonor-cloud-issue-8-3
* Merge branch 'master' into fix/sonor-cloud-issue-8-3
* Contributors: Masaya Kataoka

6.3.0 (2024-12-10)
------------------
* Merge pull request `#1468 <https://github.com/tier4/scenario_simulator_v2/issues/1468>`_ from tier4/feature/lane-change-everywhere
* fix:　disable default argument value for include_oppsite_direction in  HdMapUtils::getLeftLaneletIds/getRightLaneletIds
* Merge branch 'master' into feature/lane-change-everywhere
* fix: fit test constants to latest algorithm
* Merge remote-tracking branch 'origin/master' into feature/lane-change-everywhere
* replace true with false for include_opposite_direction option
* fix:　use lanelet2 functions correctly in HdMapUtils::getLeftLaneletIds/getRightLaneletIds
* chore: fix linelint error
* Merge remote-tracking branch 'origin/master' into feature/lane-change-everywhere
* feat: allow lane-change everywhere in `VEHICLE_WITH_ROAD_SHOULDER` routing graph
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
* Merge pull request `#1444 <https://github.com/tier4/scenario_simulator_v2/issues/1444>`_ from tier4/RJD-1057-traffic-lights-tests
  RJD-1057 (2/5): Remove non-API member functions: EntityManager’s TrafficLight related member functions (tests)
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Improve traffic lights tests subscribers
  Make the variables const and add scope for subscribers so they stop listening (only in tests with more than one subscriber + remove .reset() as scopes take care of that
* fix(traffic_simulator): fix traffic_lights test - reset subscribers
* ref(traffic_simulator): add blank line to traffic_simulator test
* ref(traffic_simulator): traffic_lights tests improve
* Use new architecture in traffic lights tests: "awf/universe/20230906" instead of "awf/universe"
* Use compile-time check instead of run-time check
* Improve traffic lights tests epsilons and threshold situation
  Add clear thresholds for tests with explanations and use these thresholds instead of hardcoded values
* Improve traffic lights tests publishing timing
  Change 'random' numbers to round ones like one second or half a second
* Improve traffic lights tests comments
* Apply patch created by Yoshimoto-san
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/RJD-1057-traffic-lights-tests' into RJD-1057-traffic-lights-tests
* Implement missing traffic lights tests and move "new architecture" tests to new file
* Rename common traffic lights tests file
* Move V2I tests to new file
* Add include guards to traffic lights test fixtures
* Move traffic light test fixtures to header file
* Add possibility to test traffic lights with different architectures
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Add missing tests for traffic lights type conversions
* Use unique pointer for returning generated traffic light messages
* Update TrafficLightPublisher and TrafficLightPublisherBase to comply with new standard
  Move message transformation to TrafficLightPublisherBase static member functions
* Merge remote-tracking branch 'tier4/master' into RJD-1057-traffic-lights-tests
* Merge branch 'master' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* fix(traffic_simulator): fix traffic_lights tests
* Merge remote-tracking branch 'origin/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge remote-tracking branch 'tier4/RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' into RJD-1057-traffic-lights-tests
* Revert "Remove traffic lights tests"
  This reverts commit 433019c50f796cfc3543922e78263fe7606b21f0.
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk, Tatsuya Yamasaki

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
* Merge pull request `#1461 <https://github.com/tier4/scenario_simulator_v2/issues/1461>`_ from tier4/entity_on_road_shoulder
  Support road shoulders as an `Entity` operating area
* refactor: add routing graph argument to RoutePlanner
* feat: change default RoutingGraphType from VEHICLE to VEHICLE_WITH_ROAD_SHOULDER
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

6.0.1 (2024-11-27)
------------------

6.0.0 (2024-11-27)
------------------
* Merge pull request `#1458 <https://github.com/tier4/scenario_simulator_v2/issues/1458>`_ from tier4/refactor/add_routing_graph_argument
* docs: add comment for HdMapUtils::DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO
* refactor: delete unused hash function
* Merge branch 'master' into refactor/add_routing_graph_argument
* chore: fix spell miss
* chore: fix build error
* chore: fix linelint error
* feat: introduce RoutingConfiguration to traffic_simulator::pose
* feat: introduce RoutingConfiguration to traffic_simulator::distance
* feat: introduce RoutingConfiguration to CanonicalizedLaneletPose
* feat: introduce RoutingConfiguration to HDMapUtils
* feat: add RoutingConfiguration
* refactor: add routing graph argument to HdMapUtils::getFollowingLanelets
* refactor: add routing graph argument to HdMapUtils::getAlongLaneletPose
* refactor: add routing graph argument to HdMapUtils::getLongitudinalDistance
* refactor: add routing graph argument to HdMapUtils::getLateralDistance
* fix: correct wrong usage of toLaneletPose
* Merge branch 'master' into refactor/add_routing_graph_argument
* refactor: add routing graph argument to HdMapUtils::getLaneChangeableLaneletId
* refactor: add routing graph argument to HdMapUtils::getSpeedLimit
* refactor: add routing graph argument to HdMapUtils::toLaneletPose(s)
* refactor: add routing graph argument to HdMapUtils::matchToLane
* refactor: add routing graph argument to HdMapUtils::getConflictingLaneIds
* refactor: add routing graph argument to HdMapUtils::countLaneChanges
* refactor: add routing graph argument to HdMapUtils::canChangeLane
* refactor: add routing graph argument to HdMapUtils::getPreviousLanelets
* Contributors: Kotaro Yoshimoto

5.5.0 (2024-11-27)
------------------

5.4.0 (2024-11-26)
------------------
* Merge pull request `#1456 <https://github.com/tier4/scenario_simulator_v2/issues/1456>`_ from tier4/feature/shoulder_routing_graph
  Add new routing graph for road shoulder
* chore: add last empty line to test_hdmap_utils.cpp
* refactor: delete unused code
* chore: add routingWithRoadShoulder test
* refactor: add routing graph argument to HdMapUtils::getRoute
* refactor: GermanRoadShoulderPassableVehicle::canPass
* refactor: add routing graph argument to HdMapUtils::get{Next|Previous}LaneletIds
* refactor: delete HdMapUtils::shoulder_lanelet
* refactor: add routing graph argument and stop giving special treatment to shoulder_lanelet in HdMapUtils::get{Next|Previous}LaneletIds
* feat: add RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER
* chore: fix cspell error
* chore: fix typo
* Merge branch 'master' into feature/shoulder_routing_graph
* feat: add new traffic rule "germanRoadShoulderPassableVehicleRules"
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

5.3.4 (2024-11-21)
------------------
* Merge pull request `#1447 <https://github.com/tier4/scenario_simulator_v2/issues/1447>`_ from tier4/fix/find_nearest_segment_index
  fix pitch and slope acceleration calculation for ego vehicle
* Merge branch 'master' into fix/find_nearest_segment_index
* feat: add tests for EgoEntitySimulation::calculateAccelerationBySlope
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

5.3.3 (2024-11-21)
------------------
* Merge pull request `#1451 <https://github.com/tier4/scenario_simulator_v2/issues/1451>`_ from tier4/fix/sonor-cloud-issue-7
  Fix/sonor cloud issue 7
* Merge branch 'master' into fix/sonor-cloud-issue-7
* fix typo
* Replace "." with "::" for access
* Contributors: Masaya Kataoka, Taiga Takano

5.3.2 (2024-11-18)
------------------
* Merge branch 'master' into fix/interpreter/assign-route-action
* Merge branch 'master' into fix/interpreter/assign-route-action
* Contributors: Tatsuya Yamasaki

5.3.1 (2024-11-18)
------------------
* Merge pull request `#1450 <https://github.com/tier4/scenario_simulator_v2/issues/1450>`_ from tier4/refactor/routing_graph
  refactor HDMapUtils class about routing graph
* Merge branch 'master' into refactor/routing_graph
* refactor: modify member function name of RoutingGraphs class
* Merge branch 'master' into refactor/routing_graph
* refactor: add RoutingGraphType argument to get{Next|Previous}LaneletIds
* refactor: move implementation of getRoute into RoutingGraphs class
* refactor: use RoutingGraphType instead of EntityType for an argument in get{Right|Left}LaneletIds
* refactor: add HdMapUtils::RoutingGraphs class to manage routing graphs
* refactor: delete operator<< and add to_string for RoutingGraphType
* feat: add routing_graph_type.hpp
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

5.3.0 (2024-11-18)
------------------
* Merge pull request `#1421 <https://github.com/tier4/scenario_simulator_v2/issues/1421>`_ from tier4/feature/manual_on_follow_trajectory_with_new_state
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
* fix(concealer): delete MANUAL control trigger via API
* Merge branch 'master' into feature/manual_on_follow_trajectory
* feat: use ADAPI for change autoware control
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge branch 'master' into feature/manual_on_follow_trajectory
* Merge remote-tracking branch 'origin/fix/RJD-955-fix-followtrajectoryaction-nan-time' into feature/manual_on_follow_trajectory
* fix: build errors
* Merge remote-tracking branch 'origin/master' into feature/manual_on_follow_trajectory
* feat: add requestAutowareControl to FieldOperatorApplicationFor class
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki

5.2.3 (2024-11-18)
------------------

5.2.2 (2024-11-15)
------------------
* Merge pull request `#1445 <https://github.com/tier4/scenario_simulator_v2/issues/1445>`_ from tier4/fix/sonor-cloud-issue-6
  Renamed lambda parameters to avoid variable shadowing.
* Merge branch 'master' into fix/sonor-cloud-issue-6
* Renamed lambda parameters to avoid variable shadowing.
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

5.2.1 (2024-11-14)
------------------
* Merge pull request `#1415 <https://github.com/tier4/scenario_simulator_v2/issues/1415>`_ from tier4/RJD-1333/previous_following_lanelets
  RJD-1333/previous following lanelets
* Resolve sonarcloud issues
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Merge branch 'master' into RJD-1333/previous_following_lanelets
* Move lambda to begginig of function
* Fix review comments
* Fix behaviour when starting lanelet is not included
* Changed imlementation to not use iterators
* Change candidate_lanelet_ids to candidate_route
* Change iterators to const
* Fix github comments
* Fix getFollowingLanelets returns wrong result
* Fix getPreviousLanelets returns wrong result
* Contributors: Grzegorz Maj, Masaya Kataoka

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
* refactor: delete unused member variable
* Merge branch 'master' into feature/traffic_light_group
* fix: fix build errors with autoware_perception_msgs/msg/traffic_light_group_array.hpp
* refactor: adjust include
* feat: support awf/universe/20240605 as architecture_type for V2I traffic light
* refactor: modify TrafficLightPublisher to publish from proto
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
* chore: apply formatter
* feat(traffic_simulator): support TrafficLightGroup for V2I traffic lights
* feat(traffic_simulator): support TrafficLightGroupArray in TrafficLightPublisher
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
* Utilize compile time check for TrafficLight
  Do not use 'default' in switch over enum to get compilation error when not all cases are considered
* Remove unused headers
* ref(traffic_simulator): simple traffic_light_publisher refactor
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* ref(traffic_simulator): remove autoware::msgs dependency, go back to a proto-centric conversion architecture
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Change variable names to be different from lambda names
  - color => color_message
  - status => status_message
  - shape => shape_message
* Use alternative operators
* Change traffic_lights_msg to traffic_lights_message
* Use explicit in traffic lights constructors
* Fix copyright notice date
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* fix(traffic_lights): improve the meaning of comments
* Fix comment style
* Fix comment
* Merge branch 'master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Add traffic light status unknown & Take advantage of compile time enum check
  Use switch without 'default' so that compiler warns when any case is not covered
* Merge remote-tracking branch 'tier4/master' into RJD-1057-remove-traffic-lights-from-entity-manager
* Rename traffic light classes to match filenames
* Revert "ref(traffic_lights): rename rest of TrafficLight**->TrafficLights**"
  This reverts commit 87bcb80e204684c476e64c9db6fa5a2bb78e6313.
* Remove traffic lights tests
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Remove exception throwing when starting a timer again in configurable rate updater
* Add TrafficLights tests + clean up other tests and verify frequency
* Rename internal traffic lights tests file
* Rename internal traffic lights tests
* Add explicit destructors for traffic lights
* Rename traffic lights test file
* Resolve problems with resources not released
* Simplify traffic lights tests
  Use pointer for traffic lights
* Use typed tests in traffic lights
* Add publishing tests for V2I traffic lights
* Add ROS message generation tests for TrafficLights
* Add message timing verification in tests
* Add conventional traffic lights tests
* feat(traffic_lights): last code improvements
* Merge branch 'RJD-1057-remove-traffic-lights-from-entity-manager' of https://github.com/tier4/scenario_simulator_v2 into RJD-1057-remove-traffic-lights-from-entity-manager
* feat(traffic_lights): use conversions instead operator() overlaod, remove overloads, improve publisher
* Remove compile error - reorder
* ref(traffic_lights): rename rest of TrafficLight**->TrafficLights**
* feat(traffic_lights): add converions for autoware_auto_perception_msgs
* feat(traffic_lights): improve traffic_lights_publisher in traffic_simulator
* feat(traffic_lights): develop cast for autoware msgs
* feat(scenario_sensor_simulator/traffic_lights_detector): remove dependencies on hdmap_utils, improve creation
* ref(traffic_simulator/traffic_lights): refactor all classes insead of TrafficLight
* feat(traffic_light_manager): use TrafficLightsBase and TrafficLights instead of TrafficLightsManager/Supervisor
* feat(traffic_lights): develop TrafficLightsBase and TrafficLights
* Merge branch 'RJD-1057-base' into RJD-1057-remove-traffic-lights-from-entity-manager
* Add TrafficLightSupervisor and move traffic lights to API
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Mateusz Palczuk, Tatsuya Yamasaki

4.5.0 (2024-11-07)
------------------
* Merge pull request `#1437 <https://github.com/tier4/scenario_simulator_v2/issues/1437>`_ from tier4/chore/extend-npc-matching-distance
* docs: add memos for default lanelet matching distance
* Merge branch 'master' into chore/extend-npc-matching-distance
* reset EntityBase::getDefaultMatchingDistanceForLaneletPoseCalculation
* use 1.5 for default lanelet matching distance for vehicle NPCs
* reset matching distance for spawning
* Merge branch 'master' into chore/extend-npc-matching-distance
* extend for default lanelet matching distance for NPCs
* Contributors: Kotaro Yoshimoto

4.4.1 (2024-11-07)
------------------
* Merge pull request `#1404 <https://github.com/tier4/scenario_simulator_v2/issues/1404>`_ from tier4/RJD-1336/fix_request_speed_change
  RJD-1336/fix_request_speed_change_throws
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* Merge branch 'RJD-1336/fix_request_speed_change' of github.com:tier4/scenario_simulator_v2 into RJD-1336/fix_request_speed_change
* add constexpr
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* add update frame to update other_statuses
* Merge branch 'master' into RJD-1336/fix_request_speed_change
* invert check, add const, add variable names
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.4.0 (2024-11-07)
------------------
* Merge pull request `#1403 <https://github.com/tier4/scenario_simulator_v2/issues/1403>`_ from tier4/fix/longitudinal_distance
  RJD-1334/fix_longitudinal_distance
* add comment
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* typo
* remove underflow
* remove negative distance, unify logic
* Merge branch 'master' into fix/longitudinal_distance
* early return, update tests
* Merge branch 'fix/longitudinal_distance' of github.com:tier4/scenario_simulator_v2 into fix/longitudinal_distance
* consider negative longitudinal distance
* Merge branch 'master' into fix/longitudinal_distance
* remove no longer used pngs
* Merge branch 'fix/longitudinal_distance' of github.com:tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'master' into fix/longitudinal_distance
* getLongitudinalDistance refactor
* name values explicitly
* add negated tests
* simplify lambda
* additional lane change check, logic simplification
* lanelet extension approach
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.3.27 (2024-11-07)
-------------------
* Merge pull request `#1438 <https://github.com/tier4/scenario_simulator_v2/issues/1438>`_ from tier4/fix/add_comment_for_empty_function
  Add comment for empty functions.
* remove warnings about misc object entity
* fix problem about https://sonarcloud.io/project/issues?open=AZJwAY5JlcZ2d3dSAT64&id=tier4_scenario_simulator_v2
* fix problems about https://sonarcloud.io/project/issues?open=AZINWTDsjvEq9OQMnL7a&id=tier4_scenario_simulator_v2
* fix warnings about https://sonarcloud.io/project/issues?open=AZJwAY5JlcZ2d3dSAT63&id=tier4_scenario_simulator_v2
* Contributors: Masaya Kataoka

4.3.26 (2024-11-06)
-------------------
* Merge pull request `#1430 <https://github.com/tier4/scenario_simulator_v2/issues/1430>`_ from tier4/fix/refactor-code
  Code Optimization and Refactoring
* Performing explicit casts.
* Removed unnecessary lambda captures.
* Added missing inline directives.
* remove unuse variable
* Contributors: Masaya Kataoka, Taiga Takano

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
* Merge pull request `#1411 <https://github.com/tier4/scenario_simulator_v2/issues/1411>`_ from tier4/RJD-1337/getQuadraticAccelerationDuration
  RJD-1337 Fix getQuadraticAccelerationDuration returning negative duration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Merge branch 'master' into RJD-1337/getQuadraticAccelerationDuration
* Fix review comments
* Fix getQuadraticAccelerationDuration returning negative duration
* Contributors: Grzegorz Maj, Masaya Kataoka

4.3.20 (2024-10-31)
-------------------
* Merge pull request `#1410 <https://github.com/tier4/scenario_simulator_v2/issues/1410>`_ from tier4/RJD-1335/requestSpeedChange
  RJD-1335 Fix requestSpeedChange sets target_speed only when it's reached
* Merge branch 'master' into RJD-1335/requestSpeedChange
* Fix requestSpeedChange sets target_speed only when it's reached
* Contributors: Grzegorz Maj, Masaya Kataoka

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
* Merge pull request `#1420 <https://github.com/tier4/scenario_simulator_v2/issues/1420>`_ from tier4/fix/negative-sqrt-in-planConstraints2
  Added checks for negative discriminants
* Added checks for negative discriminants before sqrt to prevent invalid calculations.
* Contributors: Masaya Kataoka, Taiga Takano

4.3.15 (2024-10-10)
-------------------
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
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Merge remote-tracking branch 'origin/master' into feature/faster-compilation
* Contributors: Dawid Moszyński, Shota Minami

4.3.14 (2024-10-10)
-------------------
* Merge pull request `#1409 <https://github.com/tier4/scenario_simulator_v2/issues/1409>`_ from tier4/feature/joblist-update-stand-still-duration
  Register StandStillDuration to job list
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/joblist-update-stand-still-duration
* Merge branch 'master' into feature/joblist-update-stand-still-duration
* Revised the method for obtaining time.
* fix format
* Register StandStillDuration to job list
* Contributors: Masaya Kataoka, Taiga Takano

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
* Merge pull request `#1408 <https://github.com/tier4/scenario_simulator_v2/issues/1408>`_ from tier4/feature/jpblist-update-traveled-distance
  Feature/joblist update traveled distance
* Merge branch 'master' into feature/jpblist-update-traveled-distance
* Revised the method for obtaining time.
* Merge branch 'master' into feature/jpblist-update-traveled-distance
* fixed the code error.
* remove unuse code
* fix format
* Register updateTraveledDistance to job list
* Contributors: Masaya Kataoka, Taiga Takano

4.3.10 (2024-10-03)
-------------------

4.3.9 (2024-10-03)
------------------
* Merge branch 'master' into test/cmake_flag_with_debug_and_relwithdebinfo
* Contributors: Masaya Kataoka

4.3.8 (2024-10-02)
------------------
* Merge pull request `#1389 <https://github.com/tier4/scenario_simulator_v2/issues/1389>`_ from tier4/1377/isInLanelet
  Change comparison oparator in isInLanelet
* Merge branch 'master' into 1377/isInLanelet
* Merge branch 'master' into 1377/isInLanelet
* Change comparison oparator in isInLanelet to assure result is correct for 0.0 tolerance
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
* Merge branch 'master' into RJD-1201/fix_quick_start
* Contributors: SzymonParapura

4.3.3 (2024-09-26)
------------------
* Merge pull request `#1369 <https://github.com/tier4/scenario_simulator_v2/issues/1369>`_ from tier4/fix/acquire-position-action
  Fix/acquire position action
* Merge branch 'master' into fix/acquire-position-action
* Merge branch 'master' into fix/acquire-position-action
* Merge branch 'master' into fix/acquire-position-action
* Merge remote-tracking branch 'origin/master' into fix/acquire-position-action
* Merge remote-tracking branch 'origin/master' into fix/acquire-position-action
* Update `AcquirePositionAction` to request route clearing before applying
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
* Merge pull request `#1366 <https://github.com/tier4/scenario_simulator_v2/issues/1366>`_ from tier4/RJD-1197/distance
  RJD-1197/distance
* Merge branch 'master' into RJD-1197/distance
* Merge branch 'master' into RJD-1197/distance
* resolve conflict
* Merge branch 'master' into RJD-1197/distance
* move helper functions to the helper file
* rm bug example
* BUG EXAMPLE
* remove the failing case
* distanceToRightLaneBound tests, cleanup
* distance.cpp bug fixes, distanceToRightLaneBound tests
* remove redundant constructor call
* clean-up; test_fixture
* distance tests skeleton, add intersection map and move standard map to its folder
* longitudinal distance tests
* longitudinal distance tests
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

4.2.8 (2024-09-18)
------------------
* Merge pull request `#1387 <https://github.com/tier4/scenario_simulator_v2/issues/1387>`_ from tier4/fix/set-flag-invalid-lane-pose
  fix(traffic_simulator): set flag valid lanelet pose
* fix(traffic_simulator): set flag valid lanelet pose
* Contributors: Masaya Kataoka, satoshi-ota

4.2.7 (2024-09-13)
------------------

4.2.6 (2024-09-13)
------------------
* Merge pull request `#1371 <https://github.com/tier4/scenario_simulator_v2/issues/1371>`_ from tier4/RJD-1197/pose_module
  Rjd 1197/pose module
* Change names of relativePose tests
* Fix typos in comments
* Merge branch 'master' into RJD-1197/pose_module
* CR requested changes part 2
* CR requested changes
* Pose module unit tests
* Contributors: Grzegorz Maj, Masaya Kataoka

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
* Merge pull request `#1367 <https://github.com/tier4/scenario_simulator_v2/issues/1367>`_ from tier4/RJD-1197/canonicalized_lanelet_pose
  Rjd 1197/canonicalized lanelet pose
* Spelling changes
* Lint changes
* Change assert check from bool to has_value
* Change invalid test cases to more obvious values
* Unit tests review changes
* CanonicalizedLaneletPose unit tests
* Contributors: Grzegorz Maj, Masaya Kataoka

4.2.0 (2024-09-09)
------------------
* Merge pull request `#1362 <https://github.com/tier4/scenario_simulator_v2/issues/1362>`_ from tier4/feature/ros2-parameter-forwarding
* Cleanup
* Move parameter `use_sim_time` into function `make_parameters`
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
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into fix/use-sim-time-for-real-time-factor-control
* Merge remote-tracking branch 'origin/master' into fix/use-sim-time-for-real-time-factor-control
* Merge branch 'master' into doc/RJD-1273-add-realtime-factor-doc
* use_sim_time used in concealer initialization
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Paweł Lech

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
* Merge pull request `#1358 <https://github.com/tier4/scenario_simulator_v2/issues/1358>`_ from tier4/RJD-1056-remove-npc-logic-started
  Remove unused data members: npc_logic_started
* feat(traffic_simulator): apply clang reforamt
* Merge remote-tracking branch 'origin/master' into RJD-1056-remove-npc-logic-started
* Merge branch 'RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'RJD-1056-remove-npc-logic-started' into RJD-1057-base
* Merge branch 'RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'tier4/RJD-1056-remove-npc-logic-started' into RJD-1057-base
* Merge remote-tracking branch 'tier4/RJD-1056-remove-current-time-step-time' into RJD-1057-base
* fix(traffic_simulator): remove unnecessary misc_object tests
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'origin/RJD-1056-remove-current-time-step-time' into RJD-1057-base
* Merge remote-tracking branch 'origin/RJD-1056-remove-npc-logic-started' into RJD-1057-base
* fix(ego_entity): fix autoware update when not npc_logic_started
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Corrections for adapting removed npc logic started
* Merge remote-tracking branch 'origin/ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Merge remote-tracking branch 'origin/RJD-1056-remove-npc-logic-started' into RJD-1057-base
* Correct EntityManager::getCurrentAction
* Add else
* Trailing return type
* Use member instead of getter
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-npc-logic-started
* Forward isNpcLogicStarted to API and restore TestExecutor
* Add else
* Move implementation to cpp file
* Remove npc_logic_started from API
* Update NPC logic only when it has been started
* Correct style
* Restore previous getCurrentAction behavior
* Remove npc_logic_started from Entities
* Contributors: DMoszynski, Dawid Moszynski, Masaya Kataoka, Mateusz Palczuk

4.0.2 (2024-08-28)
------------------
* Merge pull request `#1356 <https://github.com/tier4/scenario_simulator_v2/issues/1356>`_ from tier4/RJD-1056-remove-current-time-step-time
  Remove unused data members: current_time step_time
* Merge branch 'master' into RJD-1056-remove-current-time-step-time
* Fix after using pointer to store entity status
* Merge branch 'master' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Resolve conflicts with stored time
* Merge remote-tracking branch 'origin/ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Trailing return type
* Correct style - add else
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' into RJD-1056-remove-current-time-step-time
* Add const to time argument in behavior
* Add const to time argument
* Use member instead of getter
* Revert to previous macro definition
* Move requestSpeedChange time check responsibility to EgoEntity and simplify
* Correct style
* Remove step_time\_ and current_time\_ data members from EntityManager
  Adjust the code so that the time is managed in API only
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Mateusz Palczuk

4.0.1 (2024-08-28)
------------------
* Merge pull request `#1354 <https://github.com/tier4/scenario_simulator_v2/issues/1354>`_ from tier4/fix/follow_trajectory
  Fix follow trajectory action and timestamp in entity status
* Merge branch 'master' into fix/follow_trajectory
* Merge branch 'master' into fix/follow_trajectory
* Merge remote-tracking branch 'origin' into fix/follow_trajectory
* fix timestamp
* fix timestamp in status_with_trajectory
* Contributors: Masaya Kataoka, Tatsuya Yamasaki

4.0.0 (2024-08-27)
------------------
* Merge pull request `#1320 <https://github.com/tier4/scenario_simulator_v2/issues/1320>`_ from tier4/ref/RJD-1053-set-update-canonicalized-entity-status
  ref(behavior_tree, traffic_simulator): move responsibility for canonicalization to traffic_simulator, simplify
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* doc(distance): use doxygen format
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Fix EgoEntity bug where status time was incremented only when Ego was controlled by simulator
  This bug lead to problems in other checks which relied on the correct status time
* Fix EgoEntity error where map pose was unable to be set after scenario start, which should be possible for Ego
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* ref(behavior_tree): use CanonicalizedEntityStatus as shared_ptr inside BT and use ::set
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(ego_entity): fix setMapPose
* ref(traffic_simulator): remove souvenir
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(spell): fix string in api
* feat(traffic_simulator, entity_base): improve setStatus - add passing lanelet_id - use it
* ref(traffic_simulator): use getCanonicalizedStatus, remove getStatus
* ref(traffic_simulator): remove virutla getEntityType, tidy up CanonicalizedEntityStatus getters
* feat(cpp_mock_scenarios): add isPedestrain and isVehicle - use it
* ref(utils, pose): global use pose:: namespace
* fix(traffic_simulator): apply clang reformat
* ref(traffic_simulator): improve CanonicalizedEntityStatus getters - use const ref
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* ref(entity_base): fix retuned type def, fix typo
* doc(developer_guide, traffic_simulator): update doc after review changes, add code notes
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'master' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(traffic_simulator): fix helper_functions for tests and misc object tests
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge remote-tracking branch 'origin/ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(follow_trajectory_action): fix after merge FTA changes
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* doc(developer_guide, pose utils): adopt lane_pose_calculation doc to canonicalization laneletpose in CanonicalizedEntityStatus
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* Merge branch 'ref/RJD-1053-set-update-canonicalized-entity-status' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(traffic_simulator): improve assertions in CanonicalizedEntityStatus::set
* Merge branch 'ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* fix(conversions, behavior_plugin_base): add new line at the end
* ref(traffic_simulator): move toCanonicalizeLaneletPose to CanonicalizedEntityStatus::set, little tidy up
* fix(traffic_simulator): fix behavior_plugin_base
* ref(traffic_simulator, behavior_tree_plugin): revert unnecessary changes
* ref(traffic_simulator, behavior_tree_plugin): revert unnecessary changes
* ref(traffic_simulator): remove operator= for CanonicalizedEntityStatus, use set and assertions
* fix(traffic_simulator, behavior_tree_plugin): fix returned EntityStatus from bt, fix canonicalize in vehicle_entity and pedestrian_entity
* feat(behavior_tree_plugin, entity_base): move toCanonicalizedEntityPose to traffic_simulator, use EntityStatus as updated_state in BehaviorTree
* feat(traffic_simulator): use CanonicalizedEntityStatus only with single constructor
* Merge remote-tracking branch 'origin/ref/RJD-1053-implement-pose-utils' into ref/RJD-1053-set-update-canonicalized-entity-status
* tmp
* ref(traffic_simulator): remove unnecessary cast to EnrityStatus in EntityManager and MicObjectEntity
* ref(traffic_simulator): remove unecessary casts to EntityStatus in EntityBase
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Mateusz Palczuk, Tatsuya Yamasaki

3.5.5 (2024-08-27)
------------------
* Merge pull request `#1348 <https://github.com/tier4/scenario_simulator_v2/issues/1348>`_ from tier4/fix/distance-with-lane-change
  Fix longitudinal dintance calculation with lane-change in `HdMapUtils::getLongitudinalDistance`
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* chore: add a test for corner case
* Merge branch 'master' into fix/distance-with-lane-change
* Merge branch 'master' into fix/distance-with-lane-change
* fix: longitudinal calculation with lane-change
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
* Merge pull request `#1340 <https://github.com/tier4/scenario_simulator_v2/issues/1340>`_ from tier4/RJD-1278/traffic_simulator-update
  Rjd 1278/traffic simulator update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* Merge branch 'master' into RJD-1278/traffic_simulator-update
* spellcheck
* review changes
* test_traffic_light.cpp refactor, sort
* test_traffic_lights_manager.cpp refactor
* remove ros nodes
* test_simulation_clock.cpp refacton
* refactor test_helper.cpp file
* Contributors: Masaya Kataoka, Michał Ciasnocha, robomic

3.5.2 (2024-08-23)
------------------
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge branch 'master' into fix/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into fix/interpreter/user-defined-value-condition
* Contributors: Tatsuya Yamasaki, yamacir-kit

3.5.1 (2024-08-22)
------------------
* Merge pull request `#1335 <https://github.com/tier4/scenario_simulator_v2/issues/1335>`_ from tier4/feat/RJD-1283-add-traffic-controller-visualization
  feat(traffic_controller, api): add rviz marker for TrafficSink
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* ref(traffic_controller): rename make->appendDebugMarker
* Merge branch 'master' into feat/RJD-1283-add-traffic-controller-visualization
* feat(traffic_controller, api): add rviz marker for TrafficSink
* Contributors: Dawid Moszynski, Dawid Moszyński, Tatsuya Yamasaki

3.5.0 (2024-08-21)
------------------
* Merge pull request `#1316 <https://github.com/tier4/scenario_simulator_v2/issues/1316>`_ from tier4/relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* refactor: use std::size_t instead of raw size_t
  Co-authored-by: Tatsuya Yamasaki <httperror@404-notfound.jp>
* Merge branch 'master' into relative-clearance-condition
* refactor: use std::find instead of std::find_if
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge branch 'master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* Merge remote-tracking branch 'origin/master' into relative-clearance-condition
* fix tests for HdMapUtils::countLaneChanges
* Merge remote-tracking branch 'origin/relative-clearance-condition' into relative-clearance-condition
* Implement HdMapUtils::countLaneChanges
* Implement HdMapUtils::countLaneChangesAlongRoute
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
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* Merge branch 'master' into doc/longitudinal-control
* Merge branch 'master' into feat/RJD-1199-add-imu-sensor-to-simple-sensor-simulator
* feat(simulator_core, api, zmq): add attachImuSensor, add update imu sensors
* Contributors: Dawid Moszynski, Koki Suzuki, Kotaro Yoshimoto, Masaya Kataoka, SzymonParapura, koki suzuki

3.4.1 (2024-07-30)
------------------
* Merge branch 'master' into doc/open_scenario_support
* Contributors: Tatsuya Yamasaki

3.4.0 (2024-07-26)
------------------
* Merge pull request `#1325 <https://github.com/tier4/scenario_simulator_v2/issues/1325>`_ from tier4/feature/interpreter/lidar-configuration
  Feature/interpreter/lidar configuration
* Update `MiscObjectEntity` to display with a magenta bounding box
* Contributors: Masaya Kataoka, yamacir-kit

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
* Merge pull request `#1309 <https://github.com/tier4/scenario_simulator_v2/issues/1309>`_ from tier4/autoware_lanelet2_extension
  feat: use autoware_lanelet2_extension instead of lanelet2_extension
* Merge branch 'master' into autoware_lanelet2_extension
* Merge branch 'master' into autoware_lanelet2_extension
* chore: apply linter
* add autoware\_ prefix
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki, Yutaka Kondo

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
* feat(entity_manager): add Pose type assertion to ::spawnEntity
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* feat(traffic_simulator): remove deprecated test
* fix(tests): fix traffic_simulator test after merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* feat(entity_manager): remove LaneletPose support from spawnEntity()
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* feat(pose utils): apply requested changes
* fix(lanelet_pose): fix after merge - remove quaternion::
* Merge remote-tracking branch 'origin' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1053-implement-pose-utils' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1053-implement-pose-utils
* ref(pose utils, pedestrian_action_node): rename estimateCanonicalizedLaneletPose to pedestrian::transformToCanonicalizedLaneletPose
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1053-implement-pose-utils
* ref(behavior_tree_plugin, traffic_simulator): apply requested changes
* ref(traffic_simulator, pose utils): add the missing brackets
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* Merge branch 'master' into ref/RJD-1053-implement-pose-utils
* fix(traffic_simulator): fix spell
* Merge master->ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(rviz): fix last line ss2 config
* ref(rviz): add new line at the end - ss2 config
* ref(rviz): revert changes ss2 config
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* fix(entity_manager): fix reachPosition - unused target
* fix(entity_status): add operator= for CanonicalizedEntityStatus
* Revert "tmp"
  This reverts commit 6149b4cd77fa9e18ced8152c9ca0242228b5966f.
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* tmp
* ref(traffic_simulator): tidy up pose::estimateCanonicalizedLaneletPose
* ref(traffic_simulator): tidy up namespace usage, rename bbox to bounding_box
* ref(traffic_simulator): tidy up namespace usage, use inline namespace
* fix(traffic_simulator): revert rviz config changes
* ref(traffic_simulator): global improvements, comments, revert unnecessary changes
* ref(traffic_simulator): use only toMapPose and laneletLength from ::pose
* feat(traffic_simulator): use getCanonicalizedLaneletPose instead of HdMapUtils
* fix(traffic_simulator): fix map_pose after canonicalize
* feat(traffic_simulator): use consider_pose_by_road_slope as static variable in CanonicaliedLaneletPose
* ref(traffic_simulator): remove remains getMapPose
* fix(toCanonicalizeLaneletPose):  fix using unique_lanelets (current)
* fix(api, simulator_core): fix canonicalize in setEntityStatus, apply optional to canonicalize
* fix(entity_base): fix requestLaneChange
* Merge remote-tracking branch 'origin/ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* fix(toCanonicalizeLaneletPose): add use current lanelet id to provide better match
* feat(api): add throwing exception if getEntity issue durning register
* ref(simulator_core,sss,pose): revert unintended changes
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(traffic_simulator, cpp_mock_scenarios): separate getLaneletLength and adapt entire code
* ref(traffic_simulator): remove commented out for refactor
* fix(traffic_simulator): fix CanonicalizedEntityStatus with HdmapUtils constructor
* ref(traffic_simulator): remove fillLaneletPose
* ref(traffic_simulator): tidyup requestLaneChange
* ref(traffic_simulator): separate getMapPoseFromRelativePose
* ref(traffic_simulator): tidyup names and unused
* feat(traffic_simulator): add adjust Oz and pitch to CanonicalizedLaneletPose contructor
* ref(traffic_simulator, behavior_tree): separate isInLanelet, isAtEndOfLanelet, estimateLaneletPose
* ref(behavior_tree): add getLaneletId to CanonicalizedEntityStatus
* ref(traffic_simulator): tidy up constructCanonicalizedLaneletPose
* ref(traffic_simulator): improve setEntityManager - use ::pose, improve CanonicalizedEntityStatus
* ref(traffic_simulator): simply setEntityStatus - move canonicalize to EntityManager
* feat(traffic_simulator): add extend ::pose collection
* ref(traffic_simulator): adapt EntityManager to getCanonicalizedLaneletPose()
* ref(traffic_simulator): adapt pedestrian and vehicle entities to getCanonicalizedLaneletPose
* feat(traffic_simulator): add canonicalized_lanelet_pose, getCanonicalizedLaneletPose to: EntityStatus and EntityBase
* ref(traffic_simulator): use toCanonicalizedLaneletPose
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* ref(traffic_simulator): use toLaneletPose() from separated pose collection
* feat(traffic_simulator): transform PoseUtils to pose collection
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' into ref/RJD-1053-implement-pose-utils
* feat(helper): add constructCanonicalizedLaneletPose
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

2.5.0 (2024-07-08)
------------------
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge branch 'master' into feature/publish_empty_context
* Merge remote-tracking branch 'origin/master' into feature/publish_empty_context
* Contributors: Masaya Kataoka

2.4.2 (2024-07-08)
------------------

2.4.1 (2024-07-05)
------------------

2.4.0 (2024-07-01)
------------------
* Merge pull request `#1262 <https://github.com/tier4/scenario_simulator_v2/issues/1262>`_ from tier4/feature/traffic_light_for_evaluator
  Add traffic light publisher for external services
* refactor: delete unused dependencies for tarffic_simulator
* feat: use traffic_simulator_msgs for traffic light
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* refactor: use const ref
* Merge branch 'master' into feature/traffic_light_for_evaluator
* Merge branch 'master' into feature/traffic_light_for_evaluator
* chore: use conditional compilation implementation in the absence of TrafficLightArrayV1
* feat: integrate TrafficLightPublisher for evaluator to EntityManager
* feat: implement TrafficLightPublisher specialization for tier4_simulation_msgs
* Contributors: Kotaro Yoshimoto

2.3.0 (2024-06-28)
------------------
* Merge pull request `#1234 <https://github.com/tier4/scenario_simulator_v2/issues/1234>`_ from tier4/feature/synchronized_action
  Feature/synchronized action
* Merge branch 'master' into feature/synchronized_action
* chore: format miss
* chore: Update requestSynchronize function parameter name for clarity
* Update requestSynchronize function and added new test scenario.
* chore: Update requestSynchronize function to fix border distance calculation
* chore: Update requestSynchronize function to fix border distance calculation
* chore: Update requestSynchronize function signature to include target_speed parameter
* Merge commit 'c50d79fce98242d76671360029b97c166412e76f' into feature/synchronized_action
* Merge remote-tracking branch 'origin/master' into feature/synchronized_action
* Merge commit 'bf6a962e14e3e85627fca226574120cdba30080e' into feature/synchronized_action
* removed comment
* Merge commit 'bd366bce147e65d5991b62db333cf35153dd96fb' into feature/synchronized_action
* Remove unused function and update step time in EntityBase
* Add synchronized_action subdirectory and change return type of keepStepTime function
* Refactor requestSynchronize function and add keepStepTime function
* Refactor requestSynchronize function signature
* Merge commit 'b03fd92759845935be79f7ac32366848c78a2a66' into feature/synchronized_action
* Fix synchronization bug in entity_base.cpp
* add getMaxAcceleration/getMaxDeceleration function
* fix reachPosition function
* add EntityBase::reachPosition function
* remvoe ReachPosition function
* fix comment and bump version package
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronized_action
* Fix typo in comment
* Refactor entity_base.hpp and .cpp
* Merge commit '45d42a79d92c370387749ad16c10665deb42e02c' into feature/synchronized_action
* Merge branch 'master' into feature/synchronized_action
* Update entity_base.hpp
* refuctured code of synchronized action
* Merge commit '1ceb05c7206e163eb8214ceb68f5e35e7880d7a4' into feature/synchronized_action
* Fix requestLaneChange function signatures
* Fix requestLaneChange function formatting
* Merge commit 'f74901b45bbec4b3feb288c4ad86491de642f5ca' into feature/synchronized_action
* Remove unused function reachPosition
* Merge commit '8a9b141aaf6cf5a58f537781a47f66e4c305cea3' into feature/synchronized_action
* Update package version and refactor reachPosition method
* Refactor synchronization logic in EntityBase::requestSynchronize()
* Update package version and fix const correctness in entity manager and entity base
* Refactor entity_base.cpp to improve code readability
* Remove unnecessary code and include statements
* Merge branch 'master' into feature/synchronized_action
* Fix distance margin typo and update comments in entity_base.cpp
* added new line at the end of the code
* Remove debug logging statements and update function names
* Merge commit '27266909414686613cea4f9aa17162d33ecf4668' into feature/synchronized_action
* Fix lanelet target pose in synchronized action
* Merge commit 'ada77d59ffd6545105e40e88e4ad50050062a3d6' into feature/synchronized_action
* Merge commit '253fa785573217ad3a6bde882724a9e35a0c99ed' into feature/synchronized_action
* Update entity_base.hpp and synchronized_action.cpp
* Update synchronized action behavior
* Update entity synchronization logic to consider acceleration limit
* Update target lanelet poses and velocities
* 途中経過
* Refactor synchronization logic and add new API method
* Disable building of C++ mock scenarios and update requestSynchronize function
* Add requestSynchronize method to API and EntityManager and few bug fix
* tried to format
* implemented atleast functions with no error
* Fix synchronization issue in EntityBase class
* Fix distance calculation in EntityBase::getDistanceToTargetLaneletPose()
* error is not fixed but pushing today's implements
* fixed revert and errors
* Implemented a draft of getDistanceToTargetLaneletPose and requestSynchronize methods
* changed the function and made it simple
* Add draft functions for synchronized action
* Contributors: Koki Suzuki, Masaya Kataoka, hakuturu583, koki suzuki

2.2.2 (2024-06-28)
------------------

2.2.1 (2024-06-27)
------------------
* Merge pull request `#1293 <https://github.com/tier4/scenario_simulator_v2/issues/1293>`_ from tier4/fix/issue1276-re
  Optimize entity frame computation.
* Merge remote-tracking branch 'origin/master' into fix/issue1276-re
* Optimize entity frame computation.
* Contributors: Masaya Kataoka, Taiga Takano

2.2.0 (2024-06-24)
------------------
* Merge pull request `#1292 <https://github.com/tier4/scenario_simulator_v2/issues/1292>`_ from tier4/feature/clear_route_api
  add API::clearRoute()
* Merge branch 'master' into feature/clear_route_api
* Merge remote-tracking branch 'origin/master' into feature/clear_route_api
* Merge branch 'master' into feature/clear_route_api
* Merge branch 'master' into feature/clear_route_api
* add API::clearRoute()
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

2.1.11 (2024-06-24)
-------------------
* Merge pull request `#1287 <https://github.com/tier4/scenario_simulator_v2/issues/1287>`_ from tier4/feature/unit_tests/miscellaneous
  Feature/unit tests/miscellaneous
* resolve conflict
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/miscellaneous
* fix typo
* remove unecessary assignment
* add newlines
* add copy of helper_functions, apply review requests
* remove misc tests
* resolve merge confilct
* resolve merge
* remove test list file
* use test fixtures
* rename literal
* use test fixture
* remove variables that are used only once
* remove unnecesary checks
* newlines
* add tes descriptions
* prepare notes for descriptions
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/miscellaneous
* remove conversions tests
* remove tests that exist in other branches and refactor wht is left
* style
* getDistanceToLeftLaneBound
* getRouteLanelets_empty
* simulation clock tests
* entity_base tests
* entity_base tests; code cleanup
* EntityBase onUpdate. onPostUpdate tests
* entity_base dummy class with a couple of tests
* job accessor tests
* make linter happy
* job and job_list unit tests
* behavior getRequestString
* vertex, toPoints
* traffic light bulb tests
* traffic_light status shape color tests
* traffic_simulator unit tests
* Contributors: Masaya Kataoka, robomic

2.1.10 (2024-06-24)
-------------------
* Merge pull request `#1286 <https://github.com/tier4/scenario_simulator_v2/issues/1286>`_ from tier4/feature/unit_tests/misc_object_entity
  Feature/unit tests/misc object entity
* resolve conflict
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/misc_object_entity
* internal review corrections
* use better naming
* merge tests into a singular file
* clean up
* add newline
* rename file definition
* use builder
* remove variables used only once
* make it work; some tests have been deleted
* collect tests, will not compile
* Contributors: Masaya Kataoka, robomic

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
* Convert parameterHandler class to free function getParameter used with parameters interface
* getParameter -> getROS2Parameter
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Revert changes adding parameter checking
  After thic change the code is functionally the same as in the beginning
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Declare parameters before getting values
* ref(ParameterManager): rename to NodeParameterHandler, improve
* Make ParameterManager explicit
* Remove getParameter from EntityManager
* Remove getParameter from EgoEntity
* Use ParameterManager in EgoEntity
* Use const for getParameter
* Add getParameter forwarding in API
* Add ParameterManager to API
* Move ParameterManager
* Prototype ParameterManager
* Contributors: Dawid Moszynski, Masaya Kataoka, Mateusz Palczuk

2.1.6 (2024-06-18)
------------------
* Merge pull request `#1289 <https://github.com/tier4/scenario_simulator_v2/issues/1289>`_ from tier4/revert-1284-fix/issue1276
  Revert "Optimize `EntityManager::broadcastEntityTransform` to Execute Only Once"
* Revert "Optimize `EntityManager::broadcastEntityTransform` to Execute Only Once"
* Contributors: Masaya Kataoka

2.1.5 (2024-06-18)
------------------

2.1.4 (2024-06-14)
------------------
* Merge pull request `#1281 <https://github.com/tier4/scenario_simulator_v2/issues/1281>`_ from tier4/fix/remove_quaternion_operation
  Remove quaternion_operation
* Merge branch 'master' into fix/remove_quaternion_operation
* Merge branch 'master' into fix/remove_quaternion_operation
* Remove quaternion_operation
* Contributors: Masaya Kataoka, Taiga Takano

2.1.3 (2024-06-14)
------------------
* Merge pull request `#1284 <https://github.com/tier4/scenario_simulator_v2/issues/1284>`_ from tier4/fix/issue1276
  Optimize `EntityManager::broadcastEntityTransform` to Execute Only Once
* Merge branch 'master' into fix/issue1276
* fix format
* Only publsih tf onece.
* Contributors: Masaya Kataoka, Taiga Takano

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
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Contributors: Tatsuya Yamasaki, yamacir-kit

2.1.1 (2024-06-11)
------------------
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/reorder
* Contributors: Kotaro Yoshimoto, hakuturu583

2.1.0 (2024-06-11)
------------------
* Merge pull request `#1226 <https://github.com/tier4/scenario_simulator_v2/issues/1226>`_ from tier4/fix/RJD-955-fix-followtrajectoryaction-nan-time
  fix(follow_trajectory_action): adapt to work with considering slopes
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* ref(traffic_simulator, behavior_tree): apply requested changes
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
* fix(follow_trajectory): fix velocity vector and setting orientation - pitch consideration, add calc distance along lanes - if possible
* fix(toLaneletPose): fix matching_distance in EgoEntity, EgoEntitySimulation and BehaviorTree
* fix(entities_update): fix update time in EntityStatus
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Tatsuya Yamasaki

2.0.5 (2024-06-11)
------------------
* Merge pull request `#1269 <https://github.com/tier4/scenario_simulator_v2/issues/1269>`_ from tier4/feature/unit_tests/longitudinal_speed_planner
  Feature/unit tests/longitudinal speed planner
* cmakelists style fix
* use test fixture, use constexpr
* merge / resolve confict
* add comment
* remove ';' from comments
* remove variables that are only used once
* slightly alter getRunningDistance_shortTime test
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* spelling
* add reason/test description
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* change variable name
* review: use builder, rename variables; use const when possible
* add copyright notice
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* remove implicit conversion
* add newline at the end of a file
* remove problematic tests
* clarify comments, and break a test :)
* Minor test fixes
* fix epsilon
* typo
* longitudinal speed planner tests
* Contributors: Masaya Kataoka, Mateusz Palczuk, robomic

2.0.4 (2024-06-10)
------------------
* Merge pull request `#1270 <https://github.com/tier4/scenario_simulator_v2/issues/1270>`_ from tier4/feature/unit_tests/hdmap_utils
  Feature/unit tests/hdmap utils
* spellcheck
* as requested, remove variables that are only used once
* update coordinates for each map
* use test_f
* add comment on yaw angles
* update older tests desciptions
* remove ';' from comments
* use builder
* alter comment, alter distance
* update comments - unit tests will not measure performance
* remove unnecessary lines
* alter getClosestLaneletId_onlyCrosswalkNearButExcluded testcase
* fix spelling
* add test descriptions
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* fix spelling
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* apply suggestions
* remove problematic tests
* remove comments
* Merge branch 'feature/unit_tests/hdmap_utils' of github.com:RobotecAI/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* getDistanceToStopLine and getDistanceToTrafficLightStopLine tests
* Add HdMapUtils::getCenterPoints tests
* Improve stopline tests readability with additional information on failure
* Use loop in tests with vectors
* Add test expect macros with information stream
* Fix style in HdMapUtils tests
* Add comment about the issue
* Change HdMapUtils::clipTrajectoryFromLaneletIds test reference values
* Add clipTrajectoryFromLaneletIds tests
* rename symbol
* Merge branch 'feature/unit_tests/hdmap_utils' of github.com:RobotecAI/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* add map, stopLine tests
* Add getLateralDistance tests
* Add missing getAlongLaneletPose tests
* Add missing getFollowingLanelets tests
* Merge branch 'feature/unit_tests/hdmap_utils' of github.com:RobotecAI/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* getLongitudinalDistance, getTrafficLightIdsOnPath tests
* Add getPreviousLanelets tests
* Add toLaneletPose tests
* Add test expect macros
* getLaneletLength tests
* isTrafficLight, isTrafficLightRegulatoryElement tests
* getRoute tests
* fix style; canChangeLane tests
* getFollowingLanelets, getConflictingLaneIds, getConflictingCrosswalkIds test
* added 2 maps, macros, and many tests
* Add newline
* Add more HdMapUtils tests
* Style change for HdMapUtils tests
* isInRoute tests
* clarify comments regarding issues
* remaining getNextLaneletIds and getPreviousLaneletIds tests
* getNextLaneletIds tests
* getClosestLaneletId tests
* more tests
* hdmap_utils test; bug
* refactor
* update file structure
* Contributors: Masaya Kataoka, Mateusz Palczuk, robomic

2.0.3 (2024-06-10)
------------------
* Merge pull request `#1263 <https://github.com/tier4/scenario_simulator_v2/issues/1263>`_ from tier4/fix/remove_linear_algebra
  Fix/remove linear algebra
* Update simulation/traffic_simulator/src/entity/entity_manager.cpp
* fix format
* Merge branch 'master' into fix/remove_linear_algebra
* reformat
* update
* update
* Update entity_manager.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* remove linear_algebra
* Contributors: Masaya Kataoka, Taiga, Taiga Takano

2.0.2 (2024-06-03)
------------------
* Merge pull request `#1271 <https://github.com/tier4/scenario_simulator_v2/issues/1271>`_ from tier4/fix/rviz_config
  fix rviz config
* fix rviz config
* Contributors: Masaya Kataoka, hakuturu583

2.0.1 (2024-05-30)
------------------
* Merge branch 'master' into refactor/openscenario_validator
* Merge branch 'master' into refactor/openscenario_validator
* Contributors: Kotaro Yoshimoto

2.0.0 (2024-05-27)
------------------
* Merge pull request `#1233 <https://github.com/tier4/scenario_simulator_v2/issues/1233>`_ from tier4/ref/RJD-1054-implement-distance-utils
  ref(traffic_simulator): implement separate class for distance calculations, adapt make positions in SimulatorCore
* apply reformat
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator): tidy up namespaces in ::distance, ::pose
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge remote-tracking branch 'origin/master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* doc(entity_manager): add an explanation why there is no exception thrown in getEntity()
* ref(cpp_mock, simulator_core, pose): improve names
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'ref/RJD-1054-implement-distance-utils' of https://github.com/tier4/scenario_simulator_v2 into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator): tidy up pose, distance namespaces usage
* ref(traffic_simulator, pose): rename from getters to noun function name
* ref(traffic_simulator, distance): rename from getters to noun function name
* fix(traffic_simulator): fix getTimeHeadway in api
* ref(traffic_simulator, simulator_core): improve passed and returned value, fix format
* ref(traffic_simulator): remove unnecessary using and blank lines, add inline namespace - pose and distance
* ref(traffic_simulator): sort libs in CMakeLists
* ref(traffic_simulator, geometry): rename get2DPolygon to toPolygon2D, avoid abbreviation to bbox
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* ref(traffic_simulator,distance): ref getDistanceToLaneBound
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* fix(traffic_simulator): fix pose utils
* fix(traffic_simulator: fix pose and distance utils collections, improve
* fix(traffic_simulator): fix missing changes
* ref(traffic_simulator,openscenario_interpreter): improve make pose names
* ref(traffic_simulator): little format improve
* ref(traffic_simulator): remove old distance utils
* rev(traffic_simulator): remove unexpected changes - comments etc
* fix(entity_manager): add exception when try getEntity
* ref(traffic_simulator): move get quiet nan pose to pose ns
* ref(traffic_simulator): separate pose functions as namespace
* ref(traffic_simulator): transform DistanceUtils to distance namespace
* ref(traffic_simulator): tidy up distance utils, move get2DPolygon to bbox
* ref(distance) use separated getDistanceToCrosswalk and getDistanceToStopLine
* feat(distance): use separated getDistanceToBound
* feat(pose): use separated getRelativePose, makeNative**, convert and canonicalize, move poses casts definition
* feat(distance): use separated getBoundingBox..
* feat(distance): use separated getLateral and getLongitudinal distances
* feat(traffic_simulator): add getEntity, getHdmapUtils, getName
* feat(distance): init separate class for distance calc
* Contributors: DMoszynski, Dawid Moszynski, Dawid Moszyński, Masaya Kataoka, Tatsuya Yamasaki, hakuturu583

1.18.0 (2024-05-24)
-------------------
* Merge pull request `#1231 <https://github.com/tier4/scenario_simulator_v2/issues/1231>`_ from tier4/feature/traffic-source
  Feature/traffic source
* Merge branch 'master' into feature/traffic-source
* Merge branch 'master' into feature/traffic-source
* Merge remote-tracking branch 'origin/master' into feature/traffic-source
* Add comment explaining the possibly undesirable behavior
* Add comment explaining the issue
* Change lanelet fitting behavior in TrafficSource
  Enable footprint fitting to many lanelets. This allows positions on the boundary of two lanelets one after another
* Apply patched changes
* Merge branch 'master' into feature/traffic-source
* Rename unknown variables and functions for spell-check test
* Add API comment
* Optimize polygon search
  Add first search from starting lanelets which improves performance
* Add debugging function
* Fix polygon search
* Change algorithm to less greedy
* Correct redundant areas removal algorithm
* Fix compile error
* Add disable option to SpawnPoseValidator
* Prevent loops
* Fix style in TrafficSource
* First attempt at behavior and model3d support for TrafficSimulator
* Clean TrafficSource code
* Fix style
* Fix getNearbyLaneletIds not filtering out crosswalks
* Enable footprint fitting before spawning
  The solution has been optimized for a high rates, meaning combined lane polygons are precomputed when TrafficSource is added and when validating a spawn pose the polygons are only accessed
* Add require footprint fitting to API
* Enable align entity orientation to lane
* Rename API::defineTrafficSource function
* Make more readable
* Enable TrafficSource spawn rate higher than execute() rate
* Refactor TrafficSource
* Change bounding box inside circle check to 2D
* Add bounding box inside circle check
* Fix timing issues
* Fix compile error
* Rename function
* Forward arguments from API
* Fix minor helper mistake
* Add orientation and improve
* Fix function types
* Initial TrafficSource changes to randomize poses in cartesian coordinates
* Minor refactor
* Add API comment
* Fix copyright
* Rename TrafficSource member variables
* Precalculate valid S value ranges for performance improvement
* Fix HdMapUtils::getNearbyLaneletIds
  After the fix only lanelets in the specified distance are returned
* Improve TrafficSource
* Add random valid pose generation
* Refactor TrafficSource
* Add TrafficSource class
* Contributors: Masaya Kataoka, Mateusz Palczuk, Tatsuya Yamasaki

1.17.2 (2024-05-22)
-------------------

1.17.1 (2024-05-21)
-------------------
* Merge pull request `#1255 <https://github.com/tier4/scenario_simulator_v2/issues/1255>`_ from tier4/fix/visualization
  Fix/visualization
* fix orientation
* fix frame_id
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
* Merge pull request `#1245 <https://github.com/tier4/scenario_simulator_v2/issues/1245>`_ from tier4/feature/remove_entity_type_list
  remove unused member values in behavior plugin
* Merge branch 'master' into feature/remove_entity_type_list
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_entity_type_list
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_entity_type_list
* Merge remote-tracking branch 'origin/feature/remove_entity_type_list' into feature/remove_entity_type_list
* Merge branch 'master' into feature/remove_entity_type_list
* remove unused member values in behavior plugin
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
* ref(traffic_simulator, respawn): apply requested PR changes
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* ref(respawn): remove engagable() check
* ref(respawn): improve current solution
* feat(EgoEntity): add setControlledBySimulator
* fix(respawn): fix after merge - add updateFrame, adjust isEgo
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Merge branch 'master' into feature/respawn-entity
* Merge remote-tracking branch 'origin/master' into feature/respawn-entity
* Despawning and Spawning entity replaced with UpdateEntityStatusRequest
* updateFrame removed from API::respawn
* Code cleaning
* Exceptions inside API::respawn
* Merge remote-tracking branch 'origin-ssh/master' into feature/respawn-entity
* Spellcheck fix
* Code cleaning
* Respawn logic moved to API
* Interface to get vehicle parameters of entity
* RespawnEntity added
* Contributors: DMoszynski, Dawid Moszyński, Paweł Lech, Tatsuya Yamasaki

1.15.7 (2024-05-09)
-------------------
* Merge pull request `#1239 <https://github.com/tier4/scenario_simulator_v2/issues/1239>`_ from tier4/feature/speed_up_set_other_status
  Feature/speed up set other status
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/speed_up_set_other_status
* Merge remote-tracking branch 'origin/feature/publish_scenario_frame' into feature/speed_up_set_other_status
* Merge remote-tracking branch 'origin/master' into feature/speed_up_set_other_status
* Revert "remove entity status type"
  This reverts commit 17b871e35d689cb23eb4ffd1d16dbaaeade40370.
* remove entity status type
* speed up setOtherStatus logic
* Contributors: Kotaro Yoshimoto, hakuturu583

1.15.6 (2024-05-07)
-------------------
* Merge pull request `#1238 <https://github.com/tier4/scenario_simulator_v2/issues/1238>`_ from tier4/feature/publish_scenario_frame
  Publish `entities` frame
* Merge branch 'master' into feature/publish_scenario_frame
* add doxygen comment
* add doxygen comment
* Merge remote-tracking branch 'origin/feature/publish_scenario_frame' into feature/publish_scenario_frame
* use getEgoName function in broadcastEntityTransform function
* Merge branch 'master' into feature/publish_scenario_frame
* fix frame_id of visualization
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, hakuturu583

1.15.5 (2024-05-07)
-------------------

1.15.4 (2024-05-01)
-------------------

1.15.3 (2024-04-25)
-------------------
* Merge pull request `#1112 <https://github.com/tier4/scenario_simulator_v2/issues/1112>`_ from tier4/fix/standstill-duration-for-miscobjects
  Fix/Standstill duration condition for MiscObjects
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge branch 'master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* Merge remote-tracking branch 'origin/master' into fix/standstill-duration-for-miscobjects
* stanstill update for misc objects
* Contributors: Masaya Kataoka, Piotr Zyskowski

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

1.14.1 (2024-04-12)
-------------------

1.14.0 (2024-04-12)
-------------------

1.13.0 (2024-04-11)
-------------------
* Merge pull request `#1216 <https://github.com/tier4/scenario_simulator_v2/issues/1216>`_ from tier4/feature/routing-algorithm
  Implement `DistanceCondition` / `RelativeDistanceCondition` for `shortest` of `RoutingAlgorithm`
* chore: apply linter
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/feature/routing-algorithm' into feature/routing-algorithm
* doc: add comment to connect documentation
* Merge branch 'master' into feature/routing-algorithm
* Merge remote-tracking branch 'origin/master' into feature/routing-algorithm
* chore: apply linter
* Merge branch 'master' into feature/routing-algorithm
* fix: HdMapUtils::getLongitudinalDistance for adjacent lane
* feat: implement makeNativeBoundingBoxRelativeLanePosition for shortest routing algorithm
* feat: implement getLongitudinalDistance with lane change
* feat: implement getLateralDistance with lane change
* Merge branch 'master' into feature/routing-algorithm
* feat(traffic_simulator): add allow_lane_change option to getLongitudinalDistance methods
* feat(HDMapUtils): add allow_lane_change option to getRoute method
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
* return const &
* Update simulation/traffic_simulator/src/api/api.cpp
  Co-authored-by: Tatsuya Yamasaki <httperror@404-notfound.jp>
* Update simulation/traffic_simulator/src/api/api.cpp
  Co-authored-by: Tatsuya Yamasaki <httperror@404-notfound.jp>
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
  Co-authored-by: Tatsuya Yamasaki <httperror@404-notfound.jp>
* enable set jerk
* fix compile errors
* remvoe unused function definitions
* remove isEgo function and add is<EntityType> function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* Use capability query in get Vehicle/Pedestrian parameters functions.
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_behavior_plugin
* add doxygen comment
* fix typo
* enable reset behavior parameter
* add resetBehaviorPlugin function
* Merge remote-tracking branch 'origin/master' into HEAD
* cleand up a bit
* working
* Merge remote-tracking branch 'origin/master' into random-test-runner-docs-update
* Contributors: Masaya Kataoka, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki

1.8.0 (2024-03-25)
------------------
* Merge pull request `#1201 <https://github.com/tier4/scenario_simulator_v2/issues/1201>`_ from tier4/feature/set_behavior_parameter_in_object_controller
  Feature/set behavior parameter in object controller
* change <= to <
* add todo comment
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_behavior_parameter_in_object_controller
* add setVelocityLimit function
* fix rviz
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
* Merge pull request `#1208 <https://github.com/tier4/scenario_simulator_v2/issues/1208>`_ from tier4/fix/lanelet-matching-distance
  Fix/lanelet matching distance
* make API::getDefaultMatchingDistanceForLaneletPoseCalculation to the private functions
* Use correct entity parameters for lanelet matching
* Simplify code
* Use entity specific lanelet matching distance for setting entity status
* Contributors: Masaya Kataoka, Mateusz Palczuk

1.5.1 (2024-03-13)
------------------

1.5.0 (2024-03-12)
------------------
* Merge pull request `#1209 <https://github.com/tier4/scenario_simulator_v2/issues/1209>`_ from tier4/feature/ego_slope
  Consider road slope in distance measurement and entity poses
* doc: use 3 slashes to comment-out before doxygen command
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* chore: enable flag defaultly
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* fix: use consider_pose_by_road_slope flag in EntityManager::spawnEntity
* feat: add fill_pitch option to HdMapUtils::toMapPose
* fix: build error
* fix: fit WorldPosition to lanelet in spawn function
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
  # Conflicts:
  #	simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp
  #	simulation/simple_sensor_simulator/src/simple_sensor_simulator.cpp
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Merge remote-tracking branch 'origin/master' into feature/ego_slope
* Merge branch 'master' into feature/ego_slope
* update slop calculation logic
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka

1.4.2 (2024-03-01)
------------------

1.4.1 (2024-02-29)
------------------

1.4.0 (2024-02-26)
------------------
* Merge pull request `#1163 <https://github.com/tier4/scenario_simulator_v2/issues/1163>`_ from tier4/fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
  fix(follow_trajectory_action): fix cooperation with Autoware, fix speed limits
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* ref(ego_entity): apply clang
* fix(ego_entity): fix after merge
* Merge remote-tracking branch 'origin/master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* Merge branch 'master' into fix/RJD-834_fix_follow_trajectory_action_autoware_cooperation
* ref(follow_trajectory_action): apply review changes
* ref(ego_entity): revert unnecessary changes
* fix(ego_entity): remove unused arg
* ref(sss,simulation_interface, ego_entity): apply ament_clang reformat
* feat(ego_entity): provide FollowTrajectoryAction execution in EgoEntity, slight ref FollowTrajectoryAction
* fear(follow_waypoint_controller): add check if remaining_time can be rounded - inf/nan
* feat(ego_entity): update BT, overwrite only FollowTrajectoryAction
* feat(follow_trajectory): improve invalid acc exception
* Revert "feat(traffic_simulator): add zeromq to ego_entity, allow FTA, target_speed, max_speed to be set"
  This reverts commit 4d150267eea2968317684dc568150283c78d0fb0.
* ref(ego_entity_simulator, proto): review changes
* fix(follow_trajectory): add missing header
* ref(simulation): apply clang reformat
* fix(traffic_simulator): remove conflicting exception
* feat(follow_trajectory_action): waypoint passed case
* fix(follow_waypoint_controller): fix no arrival time solution
* fix(route_planner): provide waypoint setting in route_planner for FollowTrajectoryAction - VehicleEntity
* feat(traffic_simulator): add zeromq to ego_entity, allow FTA, target_speed, max_speed to be set
* ref(traffic_simulator): revert forwarding  requestFollowTrajectory
* Contributors: Dawid Moszyński, Tatsuya Yamasaki

1.3.1 (2024-02-26)
------------------
* Merge pull request `#1195 <https://github.com/tier4/scenario_simulator_v2/issues/1195>`_ from tier4/feature/split_rviz_packages
  Feature/split rviz packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/split_rviz_packages
* add include
* apply reformat
* fix package path
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
* Merge pull request `#1194 <https://github.com/tier4/scenario_simulator_v2/issues/1194>`_ from tier4/feature/default_matching_distance
  Feature/default matching distance
* Merge branch 'feature/default_matching_distance' of https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
  Co-authored-by: Kotaro Yoshimoto <pythagora.yoshimoto@gmail.com>
* Merge branch 'master' into feature/default_matching_distance
* Merge branch 'feature/default_matching_distance' of https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/default_matching_distance
* fix mathing algorithum
* simplify code
* passing matching distance for lanelet_pose_caluculation
* fix document
* fix copmile error
* add getDefaultMatchingDistanceForLaneletPoseCalculation() for vehicle entity
* add getDefaultMatchinDistance function
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

1.1.0 (2024-02-22)
------------------
* Merge pull request `#1182 <https://github.com/tier4/scenario_simulator_v2/issues/1182>`_ from tier4/feature/slope_vehicle_model
  Consider road slope in ego vehicle simulation
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/slope_vehicle_model
* Update simulation/traffic_simulator/src/entity/ego_entity.cpp
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* refactor(EgoEntitySimulation): convert lane pose matching processing to getMatchedLaneletPoseFromEntityStatus function
* doc: add notification to duplicated lane matching algorithm
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
* Merge remote-tracking branch 'origin/master' into feature/slope_vehicle_model
  # Conflicts:
  #	simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp
  #	test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py
* Contributors: Kotaro Yoshimoto, Masaya Kataoka

1.0.3 (2024-02-21)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/release_description
* Contributors: Masaya Kataoka

1.0.2 (2024-02-21)
------------------
* fix CHANGELOG
* fix CHANGELOG
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
* Doxygen note
* modify rviz settings
* remove externally_updated_status\_
* add setTwist/setAcceleration function
* add setMapPose function
* Merge remote-tracking branch 'tier/master' into feature/real-time-factor-control
* Merge pull request `#1180 <https://github.com/tier4/scenario_simulator_v2/issues/1180>`_ from tier4/add_dynamic_obstacle_stop_markers
  Add debug and virtual wall markers for dynamic_obstacle_avoidance
* fix indent
* Add debug and virtual wall markers for dynamic_obstacle_avoidance
* Merge remote-tracking branch 'tier/master' into feature/real-time-factor-control
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge pull request `#1159 <https://github.com/tier4/scenario_simulator_v2/issues/1159>`_ from tier4/revert/1096
  Revert/1096
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into revert/1096
* Changes after review
* Merge pull request `#1168 <https://github.com/tier4/scenario_simulator_v2/issues/1168>`_ from tier4/feature/get_stop_line_ids
  Feature/get stop line ids
* clean up code
* rename from "Reg Elements" to "Regulatory Elements"
* use push_back
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* use emplace_back
* use emplace_back
* use emplace_back
* reformat
* apply reformat
* fix getNextLaneletIds function
* Merge remote-tracking branch 'origin/master' into feature/real-time-factor-control
* update traffic light manager class
* add getStopLines function
* add function
* Merge pull request `#1154 <https://github.com/tier4/scenario_simulator_v2/issues/1154>`_ from tier4/cleanup/add_const_to_hdmap_utils
  Cleanup/add const to hdmap utils
* use_sim_time for openscenario_interpreter is parameterized and False by default
* Merge branch 'tier4:master' into random-test-runner-docs-update
* Revert "feat: add deleted entity to traffic simulator"
  This reverts commit ba2abf393757a53e266476fc7f4184cf495837af.
* Revert "feat: remove DELETED entity type by using internal id"
  This reverts commit a15268f290e4957fbcfce1e3c52c37de23852a4c.
* Revert "feat: invalidate status in deleted entity"
  This reverts commit b35981583909f9ddfe9587a6ea92239a7324418e.
* remove & from function arguments with lanelet::Id type
* Corrected time storage in Simulation Clock. Other minor changes
* add const to all functions in hdmap utils class
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* RViz plugin controlling real time factor value
* UpdateStepTime request for updating simple sensor simulation step_time
* Possibility of changing SimulationClock::realtime_factor during the simulation with ROS 2 topic
* realtime factor fix
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Merge remote-tracking branch 'origin/master' into fix/autoware-shutdown
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, Maxime CLEMENT, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, pawellech1, yamacir-kit

0.9.0 (2023-12-21)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into prepare/release-0.9.0
* Merge pull request `#1129 <https://github.com/tier4/scenario_simulator_v2/issues/1129>`_ from tier4/feature/RJD-716_add_follow_waypoint_controller
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Merge pull request `#1149 <https://github.com/tier4/scenario_simulator_v2/issues/1149>`_ from tier4/feature/traffic-lights-awsim-support
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* ref(follow_trajectory): revert change to ensure non-void return
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* fix(follow_trajectory): fix target_speed, little refactor
* ref(follow_trajectory): apply review changes - patch
* Merge pull request `#1033 <https://github.com/tier4/scenario_simulator_v2/issues/1033>`_ from tier4/feat/condition_groups_visualization
* clang format
* two topics publishing v2i traffic lights
* Merge remote-tracking branch 'origin/master' into feature/traffic-lights-awsim-support
* Reformatted for better readability.
* feat(traffic_simulator): add EntityManager::setTrafficLightConfidence
* update config
* update rviz config
* update rviz config
* Merge pull request `#1145 <https://github.com/tier4/scenario_simulator_v2/issues/1145>`_ from tier4/feature/random_scenario
* feat(follow_trajectory): add target_speed into consideration
* fix(follow_trajectory): remove unecessary auto
* fix(follow_waypoint_controller): fix for value type
* fix(follow_waypoint_controller): fix lambda return
* fix(spell_check): fix comment
* Merge branch 'master' into feature/RJD-716_add_follow_waypoint_controller
* feat(follow_waypoint_controller): review changes
* Merge branch 'feature/random_scenario' of https://github.com/tier4/scenario_simulator_v2 into feature/random_scenario
* Merge remote-tracking branch 'origin/master' into feature/random_scenario
* cheanged default v2i traffic lights topic
* Merge remote-tracking branch 'tier4/master' into fix/geometry-bug-fixes
* Merge pull request `#1137 <https://github.com/tier4/scenario_simulator_v2/issues/1137>`_ from tier4/fix/RJD-727_fix_orientation_for_inactivity
* fix(follow_trajectory): fix orientation for norm(v)==0
* Merge branch 'experimental/merge-master' into feature/test-geometry-spline-subspline
* ref(follow_trajectory): remove unecessary changes
* fix(follow_trajectory): fix distance precision and add exception details
* fix(follow_waypoint_controller): fix arrival tolerance
* fix(follow_waypoint_controller): fix finish tolerance
* fix(follow_waypoint_controller): add copyright
* fix(follow_waypoint_controller): fix warnings
* fix(follow_waypoint_controller): fix warnings
* feat(follow_trajectory): add follow waypoint controller
* Merge branch 'master' into fix/duplicated_nodes
* Merge pull request `#1111 <https://github.com/tier4/scenario_simulator_v2/issues/1111>`_ from tier4/feature/traffic_light_confidence
* Merge remote-tracking branch 'tier4/master' into experimental/merge-master
* fix: modify comment in traffic_light.hpp
* Merge remote-tracking branch 'origin/master' into feature/traffic_light_confidence
* fix: build errors
* Merge pull request `#1113 <https://github.com/tier4/scenario_simulator_v2/issues/1113>`_ from tier4/feature/doxygen
* Merge remote-tracking branch 'origin/master' into fix/sign-of-relative-distance
* Merge pull request `#1096 <https://github.com/tier4/scenario_simulator_v2/issues/1096>`_ from tier4/feature/deleted-entity
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp
* Update simulation/traffic_simulator/include/traffic_simulator/data_type/speed_change.hpp
* Update simulation/traffic_simulator/include/traffic_simulator/data_type/speed_change.hpp
* Update simulation/traffic_simulator/include/traffic_simulator/entity/entity_base.hpp
* add clang-format off
* refactor: reflect the reviews
* update doxyfile
* add simulation package
* add comment
* add spawn outside vehicle
* refactor: renamed 'setTrafficLightConfidence' to 'setConventionalTrafficLightConfidence'
* update Doxyfile
* fix(traffic_simulator): update setTrafficLightConfidence function to handle multiple TrafficLights
* fix(traffic_simulator): fix build errors
* feat(openscenario_interpreter): add PseudoTrafficSignalDetectorConfidenceSetAction@v1
* feat(traffic_simulator): add EntityManager::setTrafficLightConfidence
* feat(traffic_simulator): move confidence field from Bulb class to TrafficLight class
* feat: invalidate status in deleted entity
* feat: remove DELETED entity type by using internal id
* feat: add deleted entity to traffic simulator
* feat(traffic_simulator): add confidence to Bulb class
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into fix/rtc_command_action/continuous_execution
* Merge pull request `#1092 <https://github.com/tier4/scenario_simulator_v2/issues/1092>`_ from tier4/feature/control_rtc_auto_mode
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* chore: apply clang_format
* feat(traffic_simualtor): set use_foa=true to use default auto_mode settings
* Merge pull request `#1101 <https://github.com/tier4/scenario_simulator_v2/issues/1101>`_ from tier4/fix/standstill_duration
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1093 <https://github.com/tier4/scenario_simulator_v2/issues/1093>`_ from tier4/feature/RJD-614_follow_trajectory_action_pedestrian_cyclist_support
* Merge remote-tracking branch 'origin/feature/control_rtc_auto_mode' into fix/rtc_command_action/continuous_execution
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* add updateStandStillDuration(step_time) and updateTraveledDistance(step_time) when the entity failed to match lanelet
* remove debug lines
* Merge remote-tracking branch 'origin' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* Merge branch 'master' into AJD-805/baseline_update_rebased
* enable clean up entity
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge pull request `#1095 <https://github.com/tier4/scenario_simulator_v2/issues/1095>`_ from tier4/feature/freespace-distance-condition
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge branch 'master' into feature/freespace-distance-condition
* Implement getBoundingBox* functions
* Merge remote-tracking branch 'origin/master' into pzyskowski/660/ss2-awsim-connection
* code refactor
* fix spelling
* implement freespace for relative distance condition
* Init working version of DistanceCondition freespace
* Merge remote-tracking branch 'origin/master' into feature/control_rtc_auto_mode
* Merge pull request `#1090 <https://github.com/tier4/scenario_simulator_v2/issues/1090>`_ from tier4/refactor/lanelet-id
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* feat(behavior_tree): add FollowPolyline action to pedestrian
* Merge remote-tracking branch 'origin/master' into AJD-805/baseline_update_rebased
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge pull request `#1069 <https://github.com/tier4/scenario_simulator_v2/issues/1069>`_ from tier4/feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into refactor/lanelet-id
* remove debug line
* fix case
* Merge remote-tracking branch 'origin/master' into feature/lanelet2_matching_via_rosdep
* Merge pull request `#1087 <https://github.com/tier4/scenario_simulator_v2/issues/1087>`_ from tier4/feature/drop_galactic_support
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Rename `gelAllCanonicalizedLaneletPoses` to `get...`
* Cleanup
* Replace `std::vector<lanelet::Id>` with `lanelet::Ids`
* Replace `std::int64_t` with `lanelet::Id`
* Replace `LaneletId` with `lanelet::Id`
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/drop_galactic_support
* Merge pull request `#1027 <https://github.com/tier4/scenario_simulator_v2/issues/1027>`_ from tier4/feature/new_traffic_light
* chore: apply formatter
* refactor(HdMapUtils): rename functions related to traffic light
* refactor(traffic_simulator): change to a comparison method that is resistant to version changes
* refactor(traffic_simulator): delete unnecessary optimization
* fix(traffic_simulator): fix build errors
* chore(traffic_simulator): add LaneletId alias
* Merge branch 'master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* refactor: use better word "thunk" instead of callback
* refactor(traffic_simulator): use LaneletId instead of std::int64_t
* refactor(traffic_simulator)
* refactor(traffic_simulator)
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* apply reformat
* Initial version of freespace distance condition
* remove workbound for galactic
* chore: apply formatter
* refactor: TrafficLightDetectorEmulator => PseudoTrafficLightDetector
* chore: change architecture_type to awf/universe/20230906
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* apply reformat
* refactor: apply formatter with clang-format v14
* reintroduced clock publshing
* add getSValue function
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* add white space
* apply reformat
* commented clock and parts of concealer
* refactor: apply formatter
* refactor: delete debug messages
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* chore: apply formatter
* fix(TrafficLight): delete relation_id in TrafficLight class and use latest getTrafficLightRelationIDFromWayID
* fix(HDMapUtils): return all relation ids from getTrafficLightRelationIDFromWayID
* apply reformat
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Update `makeUpdatedStatus` to take a reference to `PolylineTrajectory` instead of a pointer
* apply reformat
* fix(traffic_simulator/ego): switch a parameter for new architecture_type
* fix compile error
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/RJD-96_detail_message_scenario_failure
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* refactor: delete unused code / dependencies
* feat: add new architecture_type awf/universe/2023.08
* chore: apply formatter
* refactor: delete unused lines/files
* feat(traffic_simulator): implement conversion from way_id to relation_id
* refactor(traffic_simulator): fix initialization order of member variables in ConfigurableRateUpdater
* chore: apply formatter
* fix(traffic_simulator): fix compile errors
* refactor(traffic_simulator): refactor data flow with simulation_api_schema::TrafficSignal
* refactor(traffic_simulator): refactor ConfigurableRateUpdater
* refactor(traffic_simulator): delete V2ITrafficLightPublisher and use TrafficLightPublisher
* feat(traffic_simulator): use new ConfigurableRateUpdater in TrafficLightMarkerPublisher
* feat(traffic_simulator): implement proto exporting from TrafficLightManager
* feat(traffic_simulator): Add TrafficLightPublisher
* refactor(traffic_simulator): refactor ConfigurableRateUpdater
* refactor(traffic_simulator): delete unused files
* feat(simple_sensor_simulator): add base class for TrafficLightsDetector
* fix(traffic_simulator): fix build errors
* chore(traffic_simulator): reformat
* fix(traffic_simulator): use new message in V2ITrafficLightPublisher
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* fix(traffic_simulator): fix V2ITrafficLightPublisher
* feat(simulation_interface): attachTrafficLightDetectorEmulatorRequest/Response
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* chore(traffic_light): apply ament_clang_format
* refactor(traffic_light): use new architecture_type
* chore(traffic_light): delay creating publisher
* Merge remote-tracking branch 'origin/master' into feature/new_traffic_light
* chore(traffic_light): override pure virtual function
* chore(traffic_light): delete empty files
* chore(traffic_light): adapt to variations of traffic light bulb types
* fix(entity_manager): fix errors for traffic light tests
* feat(entity_manager): use new traffic light managers in entity_manager
* feat(traffic_light): add codes for new message
* chore(traffic_light): prepare for runtime topic adaptation
* chore(traffic_light): delete TrafficLightManager type specialization
* chore(traffic_light): rename timers
* chore(traffic_light): set relation_id for new message type
* chore(traffic_light): add template for TrafficLight::Bulb conversion operator
* chore(traffic_light): fix errors
* fix(HDMapUtils): getTrafficLightRelationIDFromWayID
* chore: reformat
* feat(HDMapUtils): getTrafficLightRelationIDFromWayID
* chore(traffic_light): add perception messages for traffic light
* refactor(traffic_light): use TrafficLightBase
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* refactor(traffic_light): use updated function name
* feat(traffic_light): add TrafficLightBase and parameterize msg type of TrafficLight
* refactor(HDMapUtil): rename some functions
* Merge remote-tracking branch 'origin/master' into feature/fallback_spline_to_line_segments
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Merge remote-tracking branch 'origin/master' into feature/allow-goal-modification
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Mateusz Palczuk, Michał Kiełczykowski, Paweł Lech, Piotr Zyskowski, Tatsuya Yamasaki, f0reachARR, kyoichi-sugahara, yamacir-kit

0.8.0 (2023-09-05)
------------------
* Merge pull request `#1058 <https://github.com/tier4/scenario_simulator_v2/issues/1058>`_ from tier4/ref/RJD-553_restore_repeated_update_entity_status
* ref(entity_manager): remove test exceptions
* ref(entity_manager): revert comments format
* fix(follow_trajectory): fix division by zero
* fix(entity_manager): fix current_time update
* ref(traffic_simulator): increase readability setting time in api
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1074 <https://github.com/tier4/scenario_simulator_v2/issues/1074>`_ from tier4/fix/clock
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1024 <https://github.com/tier4/scenario_simulator_v2/issues/1024>`_ from tier4/feature/perception_ground_truth
* fix(traffic_sim): fix nonEgo update - ll2 issue
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* feat(traffic_sim): provide response processing for update of each entity type
* refactor: change property name from isEnableDetectedObjectGroundTruthDelay to detectedObjectGroundTruthPublishingDelay
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into fix/clock
* Merge pull request `#1056 <https://github.com/tier4/scenario_simulator_v2/issues/1056>`_ from tier4/feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge pull request `#1067 <https://github.com/tier4/scenario_simulator_v2/issues/1067>`_ from tier4/fix/rviz_config
* Rename API `UpdateFrameRequest::current_time` to `current_simulation_time`
* Rename member function `SimulationClock::onNpcLogicStart` to `start`
* Update `SimulationClock` to hold total frames instead of elapsed seconds
* Remove data member `SimulationClock::is_npc_logic_started\_`
* Fix a501d8b
* Lipsticks
* Remove member function `API::initialize`
* Remove data member `step_time\_` and `step_time_duration\_`
* Remove data member `SimulationClock::initialized\_`
* Remove data member `SimulationClock::initial_simulation_time\_`
* Remove member function `SimulationClock::initialize`
* Remove all arguments from class `SimulationClock` constructor
* Remove default argument from class `API` constructor
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* ref(clang): apply clang reformat
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* ref(single_sensor_simulator): apply changes requested in review
* Merge pull request `#1061 <https://github.com/tier4/scenario_simulator_v2/issues/1061>`_ from tier4/feature/traffic_simulator/follow-trajectory-action-2
* fix(rviz): rename to goal planner
* merge lidar publishing delay
* fix(traffic_sim): revert clang reformat entity_base
* Merge remote-tracking branch 'origin/master' into feature/perception_ground_truth
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* ref(traffic_simulator,sss): apply clang_reformat
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge pull request `#1018 <https://github.com/tier4/scenario_simulator_v2/issues/1018>`_ from tier4/fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Update simulation/traffic_simulator/include/traffic_simulator/helper/helper.hpp
* Update simulation/traffic_simulator/src/helper/helper.cpp
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action-2
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge pull request `#1046 <https://github.com/tier4/scenario_simulator_v2/issues/1046>`_ from tier4/fix/RJD-554_error_run_scenario_in_row
* Merge pull request `#1048 <https://github.com/tier4/scenario_simulator_v2/issues/1048>`_ from tier4/refactor/update_rviz_config
* Update `API::requestFollowTrajectory` to call ZeroMQ client
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* fix(zeromq): ensure single update ego, optimize UpdateEntityStatus
* Update `MultiClient::call` to return `Response` as return value
* revert lidar sensor delay's change
* Merge pull request `#1022 <https://github.com/tier4/scenario_simulator_v2/issues/1022>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* merge master branch
* ref(traffic_simulator): improve despawnEntities
* Merge remote-tracking branch 'origin/master' into fix/RJD-554_error_run_scenario_in_row
* Merge remote-tracking branch 'origin/master' into ref/RJD-553_restore_repeated_update_entity_status
* fix(traffic_simulator): revert assigning name to entity status
* ref(zeromq): restore repeated UpdateEntityStatus
* Merge pull request `#1054 <https://github.com/tier4/scenario_simulator_v2/issues/1054>`_ from tier4/remerge-1023
* Fix spelling
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Update simulation/traffic_simulator/config/scenario_simulator_v2.rviz
* apply distance filter for lidar_detected_entity
* Move file `data_type/follow_trajectory.[ch]pp` into directory `behavior`
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Revert "Revert "Merge pull request `#1023 <https://github.com/tier4/scenario_simulator_v2/issues/1023>`_ from tier4/feat/pointcloud_delay""
* Rename `trajectory_parameter` to `polyline_trajectory`
* Rename `FollowPolylineTrajectoryParameter` to `PolylineTrajectory`
* Add new message type `traffic_simulator_msgs::msg::PolylineTrajectory`
* Update `follow_trajectory::Parameter::base_time` to not to be `optional`
* Merge branch 'master' into feature/interpreter/sensor-detection-range
* Apply clang format
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* update rviz config
* feat(openscenario_interpreter): add despawnEntities
* Add new message type `traffic_simulator_msgs::msg::Polyline`
* Add new message type `traffic_simulator_msgs::msg::Vertex`
* Merge remote-tracking branch 'origin/master' into feat/relative_object_position
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Add missing comment from master
* Use CanonicalizedEntityStatus in do_nothing_plugin
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_fe8503' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'fix/longitudinal_distance' into fix/longitudinal_distance_fixed_master_merged
* Revert changes
* add interpreter for detection sensor range
* Merge pull request `#992 <https://github.com/tier4/scenario_simulator_v2/issues/992>`_ from tier4/fix/longitudinal_distance_fixed
* Add expected testcase output
* Use lambda function in getAllCanonicalizedLaneletPoses() function
* Add const and &
* Change comment format. Add additional testcases for checking lanelet canonicalization
* chore: apply clang-format
* Move `makeUpdatedStatus` into header `data_type/follow_trajectory.hpp`
* feat: add enable_ground_truth_delay to DetectionSensorConfiguration
* Merge branch 'fix/longitudinal_distance_fixed' into fix/longitudinal_distance_fixed_master_merged
* Remove checking if shortest route is empty
* Add comments to test. Change variables name for readibility
* Merge branch 'fix/longitudinal_distance_fixed' into fix/longitudinal_distance_fixed_master_merged
* Check if lanelet poses is empty
* Apply review feedback
* Check if an estimated lanelet pose can be canonicalized sucessfully. Remove hardcoded value
* Merge branch 'master' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_6789' into fix/longitudinal_distance_fixed_master_merged
* Merge branch 'master_4284' into fix/longitudinal_distance_fixed_master_merged
* Fix gelAllCanonicalizedLaneletPoses(). Improve unit tests
* Refactor gelAllCanonicalizedLaneletPoses(). Add unit tests for verifying canonicalization
* Apply clang-format linting
* Save information about alternative canonicalized lanelet pose. Add method to get alternative lanelet pose base on shortest route. Remove non canonicalized lanelet pose from CanonicalizedLaneletPose class.
* Save information about non canonicalized lanelet pose in CanonicalizedLaneletPose class.
* getFollowingLanelets() to the end of ret vector instead to the beginning.
* Add small offset in order to avoid returning nullopt. Pedestrian is able to stop its moving.
* Revert calculation of longitudinal distance
* rename function
* simplify bool EntityManager::isInLanelet
* manualy reformat
* use canonicalized
* remove static_cast
* use getLanletPose function
* simplify code
* manually format
* change const
* add comment
* remove unused toMapPose function
* remove getBoundingBox()
* add description
* claenup unnecessary member function
* simplify code
* use +=
* fix compile error
* use inline namespace
* add setrequest
* use getTwist function
* reduce line
* use const &
* use auto
* initialize lanelet pose
* use canonicalized value in random test runner
* use canonicalized value
* simplify code
* check name
* remove namespace
* fix compile errors
* cleanup code
* fix compile error
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* rename data type
* apply reformat
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* fix typo
* rename data type
* add comments
* change other entity status type
* change port data type
* fix canonicalize logic
* fix getting next lanelet algorithum
* passing canonicalized value into plugin
* fix typo
* use Canonicalized Values in traffic_simulator
* enable cast as geometry_msgs::msg::Pose
* fix compile error
* add getEntityType function
* simplify code
* fix reformat
* remove verbose
* use getLaneletPose function
* use getMapPose function and simplify code
* use getMapPose function
* simplify code
* use getBoundingBox function
* use geometry lib
* use getCurrentAccel/Twist function
* fix compile error
* use alias
* use alias
* use alias
* simplify reachPosition
* remove unused code
* simplifu code
* update some functions
* rename functions
* simplify code
* Simplify code
* fix route planning logic
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* add operators comparison operator for CanonicalizedLaneletPose class
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* replace type
* fix compile errors in interpretor
* fix compile errors in traffic_simulator package
* add namespace
* fix clang-format
* fix compile erros in interpretor
* fix errors in behavior tree
* fix compile errors in mock scenarios
* fix compile errors in traffic_simulator
* fix compile errors in traffic_simulator
* add API::canonicalize functions
* add CanonicalizedEntityStatus class
* remove empty line
* check the route is empty
* add lanelet pose data type
* cleanup waypoint queue when we cancel route
* Simplify branching
* Simplify branching
* fix compile error
* rename function
* Changed functions with names that do not convey the intent of implementation
* rename to route\_
* remove debug lines
* rename to canonicalizeLaneletPose function
* move to helper.hpp
* use const &
* remove unused function
* add setWaypoints function in route planner class
* remove nodiscard notation
* change route plannner from shared pointer to variable.
* add glog to the mock scenario
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* call sortAndUnique function in the getNextLaneletIds function
* remove compile errors in std::vector<std::int64_t> HdMapUtils::getPreviousLaneletIds function
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* add include
* Merge remote-tracking branch 'origin/fix/longitudinal_distance' into fix/longitudinal_distance
* Update simulation/traffic_simulator/include/traffic_simulator/hdmap_utils/hdmap_utils.hpp
* add util.hpp
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Update simulation/traffic_simulator/include/traffic_simulator/hdmap_utils/hdmap_utils.hpp
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Merge remote-tracking branch 'origin/fix/longitudinal_distance' into fix/longitudinal_distance
* add description for hard coded parameter
* add comment for hard-coded parameter
* Update simulation/traffic_simulator/src/entity/entity_manager.cpp
* Update simulation/traffic_simulator/src/entity/entity_manager.cpp
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* remove setHdMapUtils from derived class
* Merge remote-tracking branch 'origin/fix/longitudinal_distance' into fix/longitudinal_distance
* remove typo
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Update simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp
* remove definition for galactic support
* update comment for ROS_DISTRO\_* variable
* fix compile error
* fix compile error
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* fix reformat
* Merge remote-tracking branch 'origin' into fix/longitudinal_distance
* apply reformat
* update route planner logic in NPC
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge remote-tracking branch 'origin/master' into fix/longitudinal_distance
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* Merge branch 'fix/longitudinal_distance' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* remove deprecated function in humble
* fix get lanelet length ID
* fix clamp logic
* fix logic
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* fix get longitudinal distance logic
* check lanelet pose while requesting assing route
* enable check lanelet pose while requesting acquire position
* add clamp lanlet pose step while getting longitudinal distance
* add to and from pose to candidates
* fix typo
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/longitudinal_distance
* enable clamp while setting status
* enable clamp lanelet pose while set status function
* remove debug lines
* enable passing hd map utils in constructor
* enable matching with right/left lanelet
* add getLaneletPoses function
* add getLeft/RightLaneIds function to the HdMapUtils class
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Kyoichi Sugahara, Lukasz Chojnacki, Masaya Kataoka, Tatsuya Yamasaki, kosuke55, kyoichi-sugahara, yamacir-kit

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* Change comment do doxygen style
* Add const &
* Add const &
* Merge pull request `#1032 <https://github.com/tier4/scenario_simulator_v2/issues/1032>`_ from tier4/feature/update-rviz-config
* Update simulation/traffic_simulator/config/scenario_simulator_v2.rviz
* Update traffic_simulator rviz config
* renamed V2ITrafficLightManager to V2ITrafficLightPublisher
* do nothing plugin fix
* typo fix
* typo fix, unnecessary test removed
* setting rate for v2i marker
* code cleanup
* reset rviz configuration
* moved vehicle simulation to simple sensor simulator
* setting publshing rate for marker as well
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* clang format
* traffic lights cleanup
* topic configurable from v2i publishers constructor
* decoupled marker publishing from v2i publishing
* v2i manager renamed to publisher
* conventional traffic lights manager no longer used
* traffic marker publisher class filled
* traffic ligth marker publisher added
* traffic light manager passed to publishers
* Merge pull request `#998 <https://github.com/tier4/scenario_simulator_v2/issues/998>`_ from RobotecAI/pzyskowski/660/ego-entity-split
* renamed traffic light manager src file
* moved timer and publishing related fields to time class
* renamed traffic manager base filename
* renamed traffic light manager base
* introduced configured updater as an intermediate class
* trafic lights moved to simple sensor simulation in unelegant manner
* Refactor fillLaneletPose() to pure virtual
* Remove step_time parameter from EgoEntity constructor
* Add @note to comment
* Change throwing message. Remove __FILE_\_, __LINE\_\_
* traffic lights interface change; test fix
* Move fillLaneletPose() to EntityBase as virtual method. Implement fillLaneletPose() for EgoEntity
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* brought back working version with SSS (break working with AWSIM)
* Add additional comment slash
* add todo comment
* Move funcionality from EntityBase::fillLaneletPose() to EntityManager::fillEgoLaneletPose() because the funcionality is suppose to be use only by EGO entity
* Remove EgoEntity::getCurrentTwist() which implementation is the same as EntityBase::getCurrentTwist()
* Fix formatting
* Change assert() to THROW_SIMULATION_ERROR
* Change THROW_SEMANTIC_ERROR to THROW_SIMULATION_ERROR
* Change function name from refillEntityStatusWithLaneletData() to fillLaneletPose()
* Add todo to comment
* Code style fix
* Merge remote-tracking branch 'robo/pzyskowski/660/ego-entity-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#906 <https://github.com/tier4/scenario_simulator_v2/issues/906>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'master' into feature/rtc_custom_command_action
* Merge pull request `#1011 <https://github.com/tier4/scenario_simulator_v2/issues/1011>`_ from tier4/feature/do_nothing_plugin
* Update `follow_trajectory::Parameter` to hold base time
* Rename data member `Parameter<>::timing_is_absolute`
* Merge pull request `#1009 <https://github.com/tier4/scenario_simulator_v2/issues/1009>`_ from tier4/fix/hdmap_utils/get_stop_lines
* Fix code style divergence
* Update `EntityBase::requestFollowTrajectory` to throw exception
* Remove follow clothoid and NURBS trajectory action
* reintroduced entity publishing
* working changes
* fix space line
* fix space line
* add getStopLineIdsOnPath function
* changes to work only with AWSIM
* moved EES to SSS
* EES initialized in SSS
* re refilling lanelet
* pose and action status overwritten by data received over zmq
* returning updated status from sim
* ego status updated before frame update
* updateing statu sin sim function accepts status
* split ego and other entities updating
* single entity status setting
* update entities before frame update
* fix(traffic_sim): fix getRouteLanelets as a valid virtual
* lanelet2 map passing via zmq
* lanelet refill in EES
* add doNothing()
* using hdmap utils from EES
* using route from EES
* moved lanelet filling to TS api
* use passed state instead of internal state to refill lanelet id
* added hdmaputils to EES
* Merge remote-tracking branch 'robo/pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change
* entity status zmq update
* delete space
* getStopLinesOnPath() changed from private to public
* Update to properly calculate remaining time when timing is relative
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* fix(traffic_sim): fix missing Oz ll2 correction  in setAutowareState
* fix(traffic_sim): add setStatus to ego - fix missing setAutowareStatus
* feat(traffic_sim): add refill status with ll2 method
* Merge pull request `#1004 <https://github.com/tier4/scenario_simulator_v2/issues/1004>`_ from tier4/feat/v2i_custom_command_action
* pedestrian and misc object models passed
* removed uncalled status updates to EES
* model3d sent via zmq
* added unique key, pose and initial speed to the spawn vehicle
* refactor(traffic_simulator, openscenario_interpreter): unify usage timing of plural forms
* refactor(traffic_simulator, openscenario_interpreter): use reset instead of apply
* refactor(traffic_simulator, openscenario_interpreter): rename some variable & function name
* refactor(traffic_simulator): reduce the scope of the variable
* refactor(traffic_simulator): reduce the scope of the variable
* refactor(traffic_simulator): rename TrafficLightManagerBase::start into createTimer
* chore(traffic_simulator): delete unused codes
* refactor(traffic_simulator): optimize includes of v2i_traffic_light_manager.cpp
* refactor(traffic_simulator): optimize includes of conventional_traffic_light_manager.hpp
* refactor(traffic_simulator): optimize includes of v2i_traffic_light_manager.hpp
* refactor(traffic_simulator): append const to member variable of TrafficLightManagerBase
* removed updated entity entirely
* utilizing updated entity data
* map to keep entity status in sss; zmq entity update takes one entity at a time
* chore: apply linter
* chore: apply linter
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* merged UpdateSensorFrame into UpdateFrameRequest
* refactor
* chore: delete unused code
* chore: fix include guard
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* feat: implement update publish rate for V2ITrafficSignalState
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* feat(traffic_simulator): add TrafficLightManagerBase::start
* feat(traffic_simulator): implement update publish rate function for traffic lights
* refactor(traffic_simulator): forward getTrafficLights function to each type of traffic lights
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* refactor(traffic_simulator): forward getTrafficLights function to each type of traffic lights
* refactor(traffic_simulator): rename getTrafficRelation to getTrafficLights
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge pull request `#969 <https://github.com/tier4/scenario_simulator_v2/issues/969>`_ from RobotecAI/pzyskowski/660/concealer-split
* feat(traffic_simulator): add empty implementation of V2ITrafficSignalStateAction
* refactor(traffic_simulator): implement switching of traffic light managers
* chore: update traffic light manager tests
* feat(traffic_simulator): add V2ITrafficLightManager
* refactor(traffic_simulator): devide traffic light manager into 2 files
* Update `behavior_plugin` to receive Parameter via `shared_ptr`
* starting speed simplified
* initial speed fix
* initialize changed
* clang format
* spell fix
* spelling, style fixes
* clang format
* onUpdate changed to update in EES
* npc logic started logic fix
* spawn cleanup, despawn fix
* simplified ego status setting
* moved ego simulation to api
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* moved EES outside traffic_simulator namespace
* clang format
* npc_logic_started not taken into accoung in EES
* Revert "in progress"
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* clang format
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* using existing AutwoareUnvierse class template for FOAFor template
* clang format
* made concealer namespace unnecesary in FOA template parameter
* clang format
* renamed files after AutowareUser class change name
* applied AutowareUser name change to FOA
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* in progress
* extracted EES from EE
* missing rethrow
* get twist and pose without EES in EE
* using external status setting
* externaly set status
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* localization and vehicle state topics published on dedicated therad
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* setInitialState introduced
* add lanelet pose to entity status
* extracted setStateInternal
* EES setStatus uses internal state
* moved EES update to begining of EE update
* update previous and publish autoware moved
* made more fields private
* currentTwist taken from EES state
* using pose from EES in getCurrentPose
* moved status update to EES
* using state inside EES
* entity status in EES
* removed internal state from EES (ego entity simulation)
* made the status generation time equal to curren sim time
* privatized some of the ego entity simulation
* moved previous values and autoware update
* jerk taken from ego entity sim
* part of onUpdate moved to ego entity simulation
* removed spin at the end of an update, moved spin after entity base update
* getCurrentTwist used from ego_entity_simulation
* using getCurrentAccel from ego entity simulation
* setAutowareState from ego entity simulation
* getCurrentPose used from ego_entity_simulation
* requestSpeedChange used from EgoEntitySimulation
* fields from within ego entity simulation
* made it work
* ego entity simulation class
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* added small comments
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* clang format
* removed update function from user side of concelear
* setVelocityRepor, setOdometry, current_pose and current_velocity moved
* setGearSign, getsetGearCommand, getVehicleCommand separated
* acceleration, steering report and velocity moved
* removed timer for autoware update
* extracted getAcceleration
* renamde AutowareUniverse to AutowareUniverseUser
* renamed Autoware to AutowareUser
* renamed autoware to autoware_user
* concealer in main thread
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Cleanup
* Update `FollowPolylineTrajectoryAction` to receive parameter
* Add accessors for `Follow*TrajectoryAction` to `BehaviorPluginBase`
* Fix typo
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Update enumeration `traffic_simulator::behavior::Request`
* Add new behavior request `Request::FOLLOW_TRAJECTORY`
* Add new API `requestFollowTrajectory`
* Add new struct `follow_trajectory::Parameter` for behavior plugin
* Contributors: Dawid Moszynski, Dawid Moszyński, Kosuke Takeuchi, Kotaro Yoshimoto, Lukasz Chojnacki, Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki, hrjp, kosuke55, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#979 <https://github.com/tier4/scenario_simulator_v2/issues/979>`_ from RobotecAI/ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#988 <https://github.com/tier4/scenario_simulator_v2/issues/988>`_ from tier4/fix/ignore_errors_in_draw
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* return std::nullopt in optional_position function
* Merge pull request `#894 <https://github.com/tier4/scenario_simulator_v2/issues/894>`_ from tier4/fix/cleanup_code
* Merge branch 'master' into feature/interpreter/model3d-field
* ref(traffic_sim): apply ament_clang_format
* fix(traffic_sim): fix bad_optional_access - missing bulbs positions
* Merge remote-tracking branch 'origin/master' into feature/interpreter/publishing-delay
* Merge branch 'master' into fix/cleanup_code
* Revert "feat(traffic_sim): add setJerkLimit"
* Merge branch 'master' into feature/interpreter/environment
* Merge pull request `#981 <https://github.com/tier4/scenario_simulator_v2/issues/981>`_ from RobotecAI/ref/AJD-697_improve_port_management_zmq
* fix(traffic_sim): adjust pedestrian, vehicle entity to std::optional
* feat(traffic_sim): add out_of_range as job actvated in AddEntityAction functor
* Merge branch 'master' into fix/cleanup_code
* ref(traffic_sim): apply ament_clang_format
* fix(traffic_light): fix getTrafficLightBulbPosition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* ref(traffic_sim): apply ament_clang_format
* feat(traffic_sim): ensure max_jerk as variable in entity_base
* Revert "feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit"
* Merge remote-tracking branch 'origin/master' into clean-dicts
* ref(sim_interface): apply ament clang reformat
* Revert "feat(zmq): test performance"
* feat(zmq): test performance
* ref(zmq): add socket_port as rosparam
* Merge branch 'master' into feature/interpreter/model3d-field
* fix(traffic_sim): fix dynamic constraints in actioons
* feat(traffic_sim): inactivate out_of_range job for ego
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Revert "feat(traffic_sim):  ensure correct DynamicConstraints for vehicles (ego)"
* Merge pull request `#964 <https://github.com/tier4/scenario_simulator_v2/issues/964>`_ from tier4/feature/noise_delay_object
* Merge branch 'master' into feature/noise_delay_object
* Merge remote-tracking branch 'origin/master' into feature/interpreter/relative-heading-condition
* Merge pull request `#931 <https://github.com/tier4/scenario_simulator_v2/issues/931>`_ from RobotecAI/fix/get-unique-route-lanelets
* Fixed code for successful build
* get param from interpreter
* fix
* fix conflict
* add param
* Merge branch 'master' into feature/noise_delay_object
* feat(traffic_sim): add the ability to set max_jerk and max_speed via Properties
* feat(traffic_sim):  ensure correct DynamicConstraints for vehicles (ego)
* feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit
* ref(traffic_sim):  out_of_range  only for npc vehicles, add tolerance
* ref(traffic_sim): append out_of_range to job_list\_
* ref(traffic_simulator): remove out_of_range metric
* ref(traffic_sim): remove metrics except out_of_range
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge pull request `#904 <https://github.com/tier4/scenario_simulator_v2/issues/904>`_ from tier4/feature/add_setgoalposes_api
* remove member
* fix compile errors
* simplify code
* fix code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* register context_gamma_planner/VehiclePlugin
* Merge branch 'master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into fix/get-unique-route-lanelets
* Apply review comments
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#967 <https://github.com/tier4/scenario_simulator_v2/issues/967>`_ from RobotecAI/fix/AJD-655-terminates-sigint
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Change includes order
* Apply clang format
* Get unique values from vector and aplly linting
* Merge pull request `#966 <https://github.com/tier4/scenario_simulator_v2/issues/966>`_ from RobotecAI/fix/AJD-653-map-path-files
* unique lanelets in route fix
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* reformat: apply ament_clang_format
* refactor: apply ament_clang_format
* Merge pull request `#963 <https://github.com/tier4/scenario_simulator_v2/issues/963>`_ from tier4/fix/getting_next_lanelet
* remove unused const
* remove unused template
* fix(os_interp): fix abort caused by ~Interpreter
* fix(traffic_sim):  update map_path assert
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin' into fix/getting_next_lanelet
* Merge pull request `#958 <https://github.com/tier4/scenario_simulator_v2/issues/958>`_ from tier4/feature/noise_lost_object
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* add test code
* add shoulder lanelets member value
* update hdmap_utils class
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* add function of recognizing object with probability
* added param probability of lost recognition
* Merge branch 'master' into feature/noise_lost_object
* Merge pull request `#936 <https://github.com/tier4/scenario_simulator_v2/issues/936>`_ from tier4/import/universe-2437
* Delete un-intended line
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
* Merge pull request `#947 <https://github.com/tier4/scenario_simulator_v2/issues/947>`_ from tier4/emergency-state/import-933
* delete metric when the entity was despawned
* Move function def to improve readability
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* fix rviz file
* update stopping behavior
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* update test cases
* add matching distance parameter
* add test case
* add getLateralDistance function
* fix(vehicle_model): import bugfix from universe `#2595 <https://github.com/tier4/scenario_simulator_v2/issues/2595>`_
* Merge remote-tracking branch 'origin/master' into import/universe-2437
* chore(vehicle_model): sync vehicle_model with autoware.universe
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#933 <https://github.com/tier4/scenario_simulator_v2/issues/933>`_ from tier4/fix/out_of_range
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* fix(traffic_simulator): import bug fix from universe `#2437 <https://github.com/tier4/scenario_simulator_v2/issues/2437>`_
* Merge pull request `#914 <https://github.com/tier4/scenario_simulator_v2/issues/914>`_ from tier4/feature/simple_noise_simulator
* Fix wrong merge
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* add vehicle goalposes
* requestAssignRoute support setGoalPoses
* add search_count arg in getNearbyLaneletIds func
* delete metric when the entity was despawned
* Add naive implentation of `getTraveledDistance`
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Remove debug printings from `SpeedProfileAction`
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#909 <https://github.com/tier4/scenario_simulator_v2/issues/909>`_ from tier4/feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/feature/jerk_planning' into feature/interpreter/speed-profile-action
* add checking transition step for avoiding infinite loop
* use recursive call
* remove unused line
* allow 0 time constraint
* fix LINEAR transition with time constraint
* Merge remote-tracking branch 'origin/feature/jerk_planning' into feature/interpreter/speed-profile-action
* add getStatus function in job
* Add debug print to `EntityBase::resetDynamicConstraints`
* remove debug line
* check target speed reached first
* Merge branch 'feature/jerk_planning' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* add post update event
* add onPostUpdate function
* Add missing `max_speed` setting to `getDefaultDynamicConstraints`
* Fixed some error messages that didn't match the error cause
* Add a comment to the decision rationale for some parameters
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* fix typo
* fix isDecelerating/isAccelerating function
* fix setLinearAcceleration
* Revert "remove unused requestSpeedChange call"
* simplify function
* set default value as zero
* simplify LongitudinalSpeedPlanner::isAccelerating and LongitudinalSpeedPlanner::isDecelerating function
* use std::clamp
* use std::clamp
* add comment for hard coded parameter
* add comment
* add description for hard coded parameter
* fix indent
* fix indent
* remove unused requestSpeedChange call
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* update proto
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/simple_noise_simulator
* fix lane matching timing
* add onPostUpdate function
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* fix typo
* fix calculate stop distance function
* add getRunningDistance function
* fix speed planning logic
* fix typos
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* fix relative logic
* fix loop
* fix getCurrentTwist function
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* use getCurrentTwist function
* add NONE option for constraint
* add AUTO shape
* remove boost optional from getLinearJerk function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* remove debug line
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* add planning logic for quad acceleration
* enable run planing jerk
* fix plan function
* add comments
* add updateConstraintsFromJerkAndTimeConstraint function
* add getAccelerationDuration function
* enable get duration
* add error check
* add getLinearAccelerationDuration function
* fix compile errors
* add LongitudinalSpeedPlanner::getQuadraticAccelerationDurationToBound function
* Fix wrong operator
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* add getQuadraticAccelerationDuration function
* fix getDynamicStates function
* add check
* fix compile errors in traffic_simulator
* add speed planner class
* remove debug line
* add goal_poses to vehicle entity
* added ability to specify goalposes in requestAcquirePosition
* add setter for acceleration/deceleration rate
* add calculateEntityStatusUpdated function to the base class
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* add resetDynamicConstraints(); function
* fix compile errors
* add new field
* Use `std::optional` instead of `boost::optional`
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Add missing headers
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Format
* Replace boost::optional with std::optional
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Format
* Improve member signatures
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Merge branch 'master' into feature/traveled_distance_as_api
* Remove TraveledDistanceMetric
* implement `getTraveledDistance`
* Merge branch 'master' into fix_wrong_merge
* remove debug lines
* add debug line
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* add measurement job
* modify exclusive flag
* update stand still duration in job
* add UpdateAllJOb function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Merge branch 'feature/reset_acecel_in_request_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* change base class
* Contributors: Dawid Moszynski, Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, f0reachARR, hrjp, kyoichi-sugahara, yamacir-kit

0.6.7 (2022-11-17)
------------------
* Merge remote-tracking branch 'origin/master' into feat/heat_beat
* Merge pull request `#913 <https://github.com/tier4/scenario_simulator_v2/issues/913>`_ from tier4/use/autoware_github_actions
* fix(typo): threashold => threshold
* fix(typo): cansel => cancel
* Merge pull request `#908 <https://github.com/tier4/scenario_simulator_v2/issues/908>`_ from tier4/fix/traffic_simulator/horizon
* Remove parameter `max_distance` from `EntityManager::getLongitudinalDistance`
* Revert "Fix `traffic_simulator` distance measurement not working beyond 100m"
* Fix `traffic_simulator` distance measurement not working beyond 100m
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Merge pull request `#900 <https://github.com/tier4/scenario_simulator_v2/issues/900>`_ from tier4/feature/traffic_simulator/behavior-parameter
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/behavior-parameter
* Merge pull request `#901 <https://github.com/tier4/scenario_simulator_v2/issues/901>`_ from tier4/feature/speed_action_with_time
* add test scenario for relative
* add test scenario for time constraint
* Add new message type `traffic_simulator_msgs::msg::DynamicConstraints`
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* fix compile errors
* enable calculate active duration
* Merge branch 'master' into fix/interpreter/custom_command_action
* add argument for step time
* enable speed change with time constraint
* remove unused argument
* update functions
* add time constraint type
* enable run requestTargetSpeed with absolute target speed and time constraint
* add private function
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#898 <https://github.com/tier4/scenario_simulator_v2/issues/898>`_ from tier4/feature/interpreter/speed-profile-action
* Add test scenario `LongitudinalAction.SpeedProfileAction`
* Rename `DriverModel` to `BehaviorParameter`
* Update 'SpeedProfileAction' to respect attribute 'entityRef'
* Update `EntityManager::getGoalPoses` to not to receive non-const reference
* Cleanup `EntityManager::update`
* Update `EntityManager::spawnEntity` to check the number of `EgoEntity` <= 1
* Revert some changes
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Change `EntityBase::setOtherStatus` to not filter by distance
* Merge pull request `#897 <https://github.com/tier4/scenario_simulator_v2/issues/897>`_ from tier4/update/vehicle_model
* Merge pull request `#896 <https://github.com/tier4/scenario_simulator_v2/issues/896>`_ from tier4/refactor/traffic_simulator/spawn
* Delete sim_model_util.hpp
* Update vehicle model
* Delete sim_model_time_delay.hpp
* Update `EntityBase::setStatus` to return nothing
* Move entity type constructor implementations into `.cpp`
* Remove unused data member `*Entity::parameters`
* Remove virtual function `EntityBase::getVehicleParameters`
* Remove member function `EntityBase::getEntityType`
* Remove member function `EntityBase::getBoundingBox`
* Update `EntityBase::setStatus` to restore some possible missing data
* Revert some changes
* Remove data member `EntityBase::subtype`
* Update `EntityBase::getStatus` to return data member reference
* Merge remote-tracking branch 'origin/master' into feature/interpreter/priority
* Update `EntityBase::getEntityStatusBeforeUpdate` to return non-optional reference
* Lipsticks
* Update `EntityManager::getEntityStatus` to return non-optional value
* Remove member function `EntityManager::entityStatusSet`
* Remove member function `EntityBase::statusSet`
* Make `EntityBase::status\_` non-optional
* Update entity type constructors to receive `EntityStatus`
* Change `EgoEntity` constructor argument order
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Update `EntityBase` to receive `Pose` as constructor argument
* Update `VehicleEntity` and `EgoEntity` to receive `Pose` as constructor argument
* Update `PedestrianEntity` to receive `Pose` as constructor argument
* Update `MiscObjectEntity` to receive `Pose` as constructor argument
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/spawn
* Lipsticks
* Update API::spawn (VehicleEntity) to receive position
* Update `API::spawn` (PedestrianEntity) to receive position
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* Update `API::spawn` (MiscObjectEntity) to receive position
* Merge pull request `#893 <https://github.com/tier4/scenario_simulator_v2/issues/893>`_ from tier4/feature/interpreter/follow-trajectory-action-3
* Update `RoutePlanner` to store waypoints in deque instead of queue
* Update `RoutePlanner` member functions to not to copy arguments
* Remove unused data member `VehicleEntity::plugin_name`
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Update data member `EntityBase::stand_still_duration\_` to not to be optional
* Remove unused data member `EntityBase::waypoints\_
* Update member function `EntityBase::getEntityType` to be virtual
* Remove unused data member `EntityBase::type`
* Move data member `EntityBase::spline\_` into class `VehicleEntity`
* Remove unused data member `visibility\_`
* Merge remote-tracking branch 'origin/master' into refactor/test_runner
* Remove member function `EntityBase::setVerbose`
* Remove some copying
* Update `EntityBase::getCurrentAction` to not to non-const value
* Cleanup member function `EntityBase::EntityBase`
* Move some function implenentations into `.cpp`
* Move some function implementations into `.cpp`
* Merge pull request `#891 <https://github.com/tier4/scenario_simulator_v2/issues/891>`_ from tier4/feature/interpreter/follow-trajectory-action
* Move static assertions into .cpp
* Move namespace `lane_change` into new header `data_type/lane_change.hpp`
* Move namespace `speed_change` into new header `data_type/speed_change.hpp`
* Move namespace `behavior` into new header `data_types/behavior.hpp`
* Move data_types constructors into `.hpp`
* Lipsticks
* Cleanup struct `Constraint` and `RelativeTargetSpeed`
* Merge branch 'master' into feature/interpreter/priority
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge pull request `#875 <https://github.com/tier4/scenario_simulator_v2/issues/875>`_ from tier4/feature/concealer/acceleration
* Merge remote-tracking branch 'origin/master' into feature/concealer/acceleration
* Merge pull request `#823 <https://github.com/tier4/scenario_simulator_v2/issues/823>`_ from tier4/feature/start_npc_logic_api
* Delete commented out code
* Lipsticks
* Add new member function `getCurrentPose`
* Add new member function `EgoEntity::getCurrentTwist`
* Update `AutowareUniverse` to publish current acceleration
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge pull request `#863 <https://github.com/tier4/scenario_simulator_v2/issues/863>`_ from tier4/fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into fix/simple_sensor_simulator/fast_occupancy_grid
* Replace lanelet2_extension_psim with lanelet2_extension
* Merge remote-tracking branch 'origin/fix/ci_error' into feature/start_npc_logic_api
* fix Ego dynamics calculation
* Merge branch 'master' into feature/occupancy_grid_docs
* apply reformat
* does not update stand still duration while npc logic was not started
* Update `Interpreter` to set `Configuration::initialize_duration` to zero
* remove boost::optional value
* return scenario time
* remove early return
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* fix compile errors
* initalize with NAN
* use boost optional
* use boost::optional type
* remove passing current time to plugin
* modify constructo for stop watch class
* skip update npc status before starting npc logics
* add isNpcLogicStarted function
* fix compile errors
* add startNpcLogic API
* add startNpcLogic function
* add new member value
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge pull request `#854 <https://github.com/tier4/scenario_simulator_v2/issues/854>`_ from tier4/feature/remove_simple_metrics
* Merge pull request `#864 <https://github.com/tier4/scenario_simulator_v2/issues/864>`_ from tier4/feature/improve_ego_route_matching
* apply reformat
* enable retry toLaneletPose
* Remove CollisionMetric and StandstillMetric
* Remove CollisionMetric and StandstillMetric
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge pull request `#850 <https://github.com/tier4/scenario_simulator_v2/issues/850>`_ from tier4/feature/lanelet2_extension_psim
* Copy `lanelet2_extension` of Autoware.Universe 0.3.7 as `lanelet2_extension_psim`
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'origin/master' into doc/6th_improvement
* Merge pull request `#837 <https://github.com/tier4/scenario_simulator_v2/issues/837>`_ from tier4/update/rviz_display
* Fix runtime errors
* Fix the way to import rviz config path
* Merge remote-tracking branch 'tier/master' into fix/obstacle_detection_raycaster
* Update scenario_simulator_v2.rviz
* Merge pull request `#836 <https://github.com/tier4/scenario_simulator_v2/issues/836>`_ from tier4/fix/trajectory_offset
* Merge branch 'master' into fix/trajectory_offset
* Pass rviz_config to autoware_launch
* Merge pull request `#834 <https://github.com/tier4/scenario_simulator_v2/issues/834>`_ from tier4/fix/yield_action
* remove right of way when the lanelet id is same
* add getLaneletPose API
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#816 <https://github.com/tier4/scenario_simulator_v2/issues/816>`_ from tier4/feature/geometry_lib
* fix include guard
* fix lint error
* fix namespavce
* modify namespace
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* move directory
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge pull request `#805 <https://github.com/tier4/scenario_simulator_v2/issues/805>`_ from tier4/doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Fix spells
* Merge branch 'feature/geometry_lib' of https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* add intersection directory
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* move directory
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge pull request `#809 <https://github.com/tier4/scenario_simulator_v2/issues/809>`_ from tier4/feature/get_relative_pose_with_lane_pose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* add getRelativePose between LaneletPose and entity name
* add const
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* use std::transform
* split directory
* enable pass compile in traffic simulator
* change namespace
* apply reformat
* Merge branch 'feature/get_distance_to_lane_bound' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Update custom_spell.json
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* add geometry_math package
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* fix dependency
* Merge branch 'master' into fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* Merge pull request `#807 <https://github.com/tier4/scenario_simulator_v2/issues/807>`_ from tier4/feature/get_distance_to_lane_bound
* fix(traffic_simulator): modify build error in humble
* fixed const-non-reference warror
* traffic simulator pluginlib dependency
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_lane_bound
* Merge pull request `#803 <https://github.com/tier4/scenario_simulator_v2/issues/803>`_ from tier4/feature/replace_dummy_ogm_map
* Merge pull request `#796 <https://github.com/tier4/scenario_simulator_v2/issues/796>`_ from tier4/refactor/concealer/virtual-functions
* fix problem in push back
* add getDistanceToLaneBound function
* remove const
* update get distance to bound function
* enable call without specify lanelet id
* add definition to manager class
* add getLaneletPose function to the base class
* add definition in API class
* add getLeft/Right bound function to entity base class
* remove metric
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pull_over_metrics
* Merge branch 'master' into feature/change_engage_api_name
* Fix local function `everyone_engageable`
* Merge pull request `#779 <https://github.com/tier4/scenario_simulator_v2/issues/779>`_ from adamkrawczyk/build/add_missing_depend
* feat!: replace dummy ogm
* add judge algorithum for pull over metric
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* enable check yaw angle
* Remove member function `API::engage` and `API::ready`
* add activateTrigger function
* add getLeftBound/getRightBound function
* Remove some member functions for Autoware.Universe from API
* Merge pull request `#797 <https://github.com/tier4/scenario_simulator_v2/issues/797>`_ from tier4/feature/occupancy_grid_sensor
* Add new member function `asAutoware`
* add pull over metric class
* Merge pull request `#778 <https://github.com/tier4/scenario_simulator_v2/issues/778>`_ from tier4/feature/zmqpp_vendor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/arrange_docs_and_fix_copyright
* Merge pull request `#795 <https://github.com/tier4/scenario_simulator_v2/issues/795>`_ from tier4/fix/lane_matching_logic
* remove unused comment
* Merge remote-tracking branch 'origin/master' into doc/arrange_docs_and_fix_copyright
* Fix misses
* Fix Licence
* Add virtual function `getTurnIndicatorsCommand` to class `Autoware`
* Add virtual function `getGearCommand` to class `Autoware`
* add nan check
* change logics for checking solution
* Merge pull request `#750 <https://github.com/tier4/scenario_simulator_v2/issues/750>`_ from tier4/fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge pull request `#792 <https://github.com/tier4/scenario_simulator_v2/issues/792>`_ from tier4/fix/autoware/reverse-gear
* Fix Autoware.Universe to accept `GearCommand` correctly
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* add autoware->rethrow() in EgoEntity::onUpdate()
* Merge branch 'master' into feature/zmqpp_vendor
* Merge pull request `#785 <https://github.com/tier4/scenario_simulator_v2/issues/785>`_ from tier4/doc/improve
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/improve
* apply reformat
* rename topic
* Fix old "TierIV" annotation
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge pull request `#777 <https://github.com/tier4/scenario_simulator_v2/issues/777>`_ from tier4/feature/indicator_condition
* Add traffic sim missing depend
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* add client tot the API class
* Refactor
* Rename **StateString to **StateName
* modify CMakeLists.txt
* Implement TurnIndicatorsState as an UserDefinedValueCondition
* Merge pull request `#710 <https://github.com/tier4/scenario_simulator_v2/issues/710>`_ from RobotecAI/AJD-331-optimization-2nd-stage
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* rename CatmullRomInterface -> CatmullRomSplineInterface
* Merge pull request `#760 <https://github.com/tier4/scenario_simulator_v2/issues/760>`_ from tier4/feature/emergency_state_for_fault_injection
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge remote-tracking branch 'origin/master' into feature/emergency_state_for_fault_injection
* Revised the scope of responsibility for each getEmergencyStateString function
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Do check dynamic_cast to EgoEntity in isEgo function
* Add getEmergencyStateString function to EntityBase class
* Add dynamic_cast check to EntityManager::isEgo function
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* refactor
* Use common::Error instead of SEMANTIC_ERROR
* Refactor
* Add EmergencyStateString interface to traffic_simulator API
* Add EmergencyStateString interface on EntityManager
* Fix compile errors
* [WIP] add EmenrgencyState interface on EgoEntity
* Add semantic error
* Add shift operator overload for EmergencyState to correspond boost::lexical_cast
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Refactor
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* cleanup comments
* fix catmull rom spline unit tests
* calculate subspline from spline; hdmap_utils use spline instead of recalculating it
* create getSubspline() method for CatmullRomSpline
* Contributors: Adam Krawczyk, Daisuke Nishimatsu, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Yuma Nihei, danielm1405, kyabe2718, taikitanaka3, tanaka3, wep21, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge pull request `#761 <https://github.com/tier4/scenario_simulator_v2/issues/761>`_ from tier4/refactor/undef_mistake
* Merge pull request `#757 <https://github.com/tier4/scenario_simulator_v2/issues/757>`_ from tier4/feature/speed_up_get_length
* Fix undef macro target
* add image and doxygen comment
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/speed_up_get_length
* Merge pull request `#759 <https://github.com/tier4/scenario_simulator_v2/issues/759>`_ from tier4/feature/traffic_simulator/traffic_light
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/traffic_light
* Merge pull request `#752 <https://github.com/tier4/scenario_simulator_v2/issues/752>`_ from tier4/feature/reset_acecel_in_request_speed_change
* use std::abs instead of std::fabs
* remove if line
* Update class `TrafficLight` to not to check given ID is relation ID
* Update TrafficLight::Bulb value to be more redundant
* Update `TrafficLightManager::getTrafficLight` to not to check if given ID is traffic relation
* add EXPECT_DECIMAL_EQ
* remove old implementation
* Ignore terms above the second order of delta s
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge pull request `#740 <https://github.com/tier4/scenario_simulator_v2/issues/740>`_ from tier4/refactor/traffic_simulator/traffic_light_manager
* add comment
* modify onUpdate function
* Support old status name `Blank`
* Update `TrafficLight::Color` and `Status` to accept old names
* Lipsticks
* remove debug lines
* remove debug line
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Rename member function `getTrafficRelation` to `getTrafficRelationReferees`
* Cleanup
* Add new member function `TrafficLight::getTrafficRelation`
* Add new member function `TrafficSignal::set`
* Merge pull request `#712 <https://github.com/tier4/scenario_simulator_v2/issues/712>`_ from tier4/fix/object-recognition-from-prediction-to-detection
* Merge pull request `#751 <https://github.com/tier4/scenario_simulator_v2/issues/751>`_ from tier4/feature/behavior_request_enum
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Remove enumeration `TrafficLightColor` and `TrafficLightArrow`
* Switch struct `TrafficLight` to experimental version
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Merge pull request `#749 <https://github.com/tier4/scenario_simulator_v2/issues/749>`_ from tier4/feature/job_system
* Remove member function `TrafficLight::getPosition`
* enable reset acceleration limit
* Merge branch 'feature/behavior_request_enum' into feature/reset_acecel_in_request_speed_change
* fix compile errors
* Remove member function `TrafficLight::update`
* fix namespace
* use enum
* Lipsticks
* Update experimental TrafficLight constructor to receive ID and map
* Update class `TrafficLight` to accept only valid ID
* Update experimental traffic light to be publishable
* remove debug line
* remove debug line
* Add experimental class `TrafficLight\_`
* move to base class
* rename members
* add inactivate function
* add status
* fix compile error
* remove targetSpeedPlanner class
* enable resolve relative value
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/job_system
* Merge pull request `#690 <https://github.com/tier4/scenario_simulator_v2/issues/690>`_ from RobotecAI/AJD-331-make-zmq-client-work-through-network
* add Job and JobList class
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#744 <https://github.com/tier4/scenario_simulator_v2/issues/744>`_ from tier4/feature/remove_color_names_function
* Lipsticks
* Fix test `test_traffic_light_manager`
* Update `TrafficLightManager` to not to instantiate all TrafficLight
* Add new member function `HdMapUtils::isTrafficLight`
* Remove enumeration `TrafficLightColor::NONE`
* Merge branch 'master' into fix/interpreter/interrupt
* change license (MIT licensed code was removed)
* remove functions in color_names package
* Remove member function `TrafficLightManager::get(Arrow|Color)`
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Lipsticks
* Remove member function `TrafficLightManager::getIds` and `getInstance`
* Lipsticks
* Update `TrafficLight` contructor to locate bulb position by itself
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Fix `TrafficLight::(arrow|color)_changed\_`
* Remove header `traffic_simulator/traffic_lights/traffic_light_phase.hpp`
* Remove member function `TrafficLightPhase::(get|set)State`
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge pull request `#718 <https://github.com/tier4/scenario_simulator_v2/issues/718>`_ from tier4/fix/waypoint_height
* Remove member function `TrafficLightPhase::update`
* Remove data member `TrafficLightPhase::elapsed_time\_`
* Remove member function `TrafficLightPhase::getPhaseDuration`
* Remove member function `TrafficLightPhase::setPhase`
* Remove member function `TrafficLightPhase::getPhase`
* Remove member function `TrafficLight::get(Arrow|Color)PhaseDuration`
* Remove member function `setTrafficLightManager::set(Arrow|Color)Phase`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge pull request `#733 <https://github.com/tier4/scenario_simulator_v2/issues/733>`_ from tier4/feature/improve_ego_lane_matching
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge remote-tracking branch 'origin/master' into feature/interpreter/object-controller
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#708 <https://github.com/tier4/scenario_simulator_v2/issues/708>`_ from RobotecAI/AJD-331-optimization
* use dynamic_cast
* add getPath function
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* fix typo
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* fix height matching algorithum
* enable matching to on route lane
* Merge pull request `#729 <https://github.com/tier4/scenario_simulator_v2/issues/729>`_ from tier4/feature/ignore_raycast_result
* add getRouteLanelets function
* configure matching function
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* update error message
* rename data field
* set default value
* enable filter by range
* update proto
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* rename to subtype
* Merge branch 'feature/semantics' of https://github.com/tier4/scenario_simulator_v2 into feature/semantics
* remove category
* fix typo and remove debug line
* add disconect() to ~Interpreter(). stop zeromq call if shut down.
* remove debug line
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* apply reformat
* fix error
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* remove old method
* enable pass compile in traffic_simulator package
* add entity semantics to the member variable
* enable filter by range
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* set z value in getPoint function
* review changes: common_spline -> reference_trajectory
* Merge branch 'master' into AJD-331-optimization
* fix ci test
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* add option to disable traffic light module
* fix tests
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* clang format
* change to option 1 (simulator host as a launch parameter)
* calculate spline only when route_lanelets change
* calculate and pass common_spline
* optimize hermite curve
* internal review fixes
* zmq client can connect through the network
* Contributors: Daniel Marczak, HansRobo, Masaya Kataoka, MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge pull request `#716 <https://github.com/tier4/scenario_simulator_v2/issues/716>`_ from tier4/dependency/remove-lexus-description
  Remove `lexus_description` from dependency
* Cleanup
* Merge pull request `#714 <https://github.com/tier4/scenario_simulator_v2/issues/714>`_ from tier4/fix/get_longitudinal_distance
  Fix/get longitudinal distance
* fix typo
* remove unused lines
* change hard coded parameter
* remove debug line
* configure parameter
* add getLongitudinalDistance function in world frame
* Merge pull request `#709 <https://github.com/tier4/scenario_simulator_v2/issues/709>`_ from tier4/feature/waypoint_offset
  Feature/waypoint offset
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* remove debug line
* erase not unique waypoint
* Merge pull request `#706 <https://github.com/tier4/scenario_simulator_v2/issues/706>`_ from tier4/fix/hard_coded_parameter
  fix hard coded parameter in hermite curve class
* Merge pull request `#707 <https://github.com/tier4/scenario_simulator_v2/issues/707>`_ from tier4/fix/sim_model_delay_steer_acc_geared
  fix sim_model_delay_steer_acc_geard model
* modify getPoint function
* add offset
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* add white line at the EOF
* fix typo and add new line at EOF
* add offset in waypoint calculation
* Merge pull request `#704 <https://github.com/tier4/scenario_simulator_v2/issues/704>`_ from tier4/feature/autoware-external-api
  Feature/autoware external api
* fix sim_model_delay_steer_acc_geard model
* fix hard coded parameter in hermite curve class
* Rename member function `setUpperBoundSpeed` to `setVelocityLimit`
* Replace `AwapiAutowareStatus` with `autoware_auto_system_msgs::msg::AutowareState`
* Merge pull request `#698 <https://github.com/tier4/scenario_simulator_v2/issues/698>`_ from tier4/fix/idead_steer_acc_geard
  fix sim model ideal steer acc geard
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* fix sim model ideal steer acc geard
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Remove legacy vehicle model types
* Remove architecture_type `awf/auto`
* Remove class `AutowareAuto`
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#688 <https://github.com/tier4/scenario_simulator_v2/issues/688>`_ from tier4/fix/traffic_simulator/traffic_light_manager
  Fix/traffic simulator/traffic light manager
* Cleanup
* Merge pull request `#686 <https://github.com/tier4/scenario_simulator_v2/issues/686>`_ from tier4/fix/warp_problem
  Fix/warp problem
* Fix traffic signal topic name (for Autoware.Universe)
* Update `TrafficLightManager::set*` to be traffic relation ID acceptable
* Merge pull request `#689 <https://github.com/tier4/scenario_simulator_v2/issues/689>`_ from tier4/feature/add-occlusion-spot-no-stopping-area-marker
  feat(behavior_velocity): add occlusion spot no stopping area marker
* fix typo
* remove debug line
* fix compile errors
* lane matching fails when the offset overs 1
* Add new member function `HdMapUtils::isTrafficRelationId`
* Lipsticks
* Merge remote-tracking branch 'origin/master' into fix/traffic_simulator/traffic_light_manager
* Lipsticks
* feat(behavior_velocity): add occlusion spot no stopping area marker
* add std::sqrt function in offset calculation
* enable get - offset value
* Merge pull request `#684 <https://github.com/tier4/scenario_simulator_v2/issues/684>`_ from tier4/fix/virtual_destructor
  Fix/virtual destructor
* use override in entity class
* use override in metrics
* Merge pull request `#683 <https://github.com/tier4/scenario_simulator_v2/issues/683>`_ from tier4/feature/zeromq_multi_client
  Feature/zeromq multi client
* add virtual destructor to the metric class
* modify destructor
* add virtual destructor to the entity class
* use multi client class
* Merge pull request `#680 <https://github.com/tier4/scenario_simulator_v2/issues/680>`_ from tier4/feature/speed_up_metrics_manager
  Feature/speed up metrics manager
* remove unused lines
* remove log output step in every update frame
* remove output to file step
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge branch 'master' into fix/interpreter/lifecycle
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Replace `VehicleCommand` with `AckermannControlCommand` and `GearCommand`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Remove package `autoware_perception_msgs`
* Replace `autoware_debug_msgs` with `tier4_debug_msgs`
* Remove architecture_type `tier4/proposal`
* Remove class `AutowareArchitectureProposal`
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, tanaka3, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge pull request `#671 <https://github.com/tier4/scenario_simulator_v2/issues/671>`_ from tier4/fix/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/lane_change_trajectory_shape
* Merge pull request `#670 <https://github.com/tier4/scenario_simulator_v2/issues/670>`_ from tier4/feature/request_speed_change_in_pedestrian
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change_in_pedestrian
* Merge branch 'master' into feature/request_speed_change_in_pedestrian
* Merge pull request `#668 <https://github.com/tier4/scenario_simulator_v2/issues/668>`_ from tier4/feature/interpreter/lane-change-action
* Merge remote-tracking branch 'origin/master' into feature/interpreter/lane-change-action
* modify trajectory tangent size
* fix problem in passing driver model in pedestrian behavior plugin
* Merge pull request `#669 <https://github.com/tier4/scenario_simulator_v2/issues/669>`_ from tier4/refactor/add_speed_change_namespace
* rename functions
* add whitespace
* specify class
* fix compile error
* add speed_change namespace
* Merge remote-tracking branch 'origin/master' into feature/interpreter/lane-change-action
* Update some syntaxes to support conversion operator
* Merge pull request `#667 <https://github.com/tier4/scenario_simulator_v2/issues/667>`_ from tier4/feature/control_from_relation_id
* Merge https://github.com/tier4/scenario_simulator.auto into feature/control_from_relation_id
* remove const
* Merge pull request `#665 <https://github.com/tier4/scenario_simulator_v2/issues/665>`_ from tier4/feature/interpreter/speed-action
* add relation id
* apply reformat
* Update some structures to support cast operator
* Merge remote-tracking branch 'origin/master' into feature/interpreter/speed-action
* Merge pull request `#664 <https://github.com/tier4/scenario_simulator_v2/issues/664>`_ from tier4/feature/lateral_velocity_constraint
* Update `EgoEntity` to override `EntityBase::requestSpeedChange`
* apply reformat
* add new test scenario
* modify scenario
* add TIME constraint
* fix typo
* use switch
* add default value
* split NPC logic by using constraint type
* fix logic in calculating along pose
* add test case
* add new API
* Merge pull request `#662 <https://github.com/tier4/scenario_simulator_v2/issues/662>`_ from tier4/fix/rename_trajectory
* rename data field and remove unused field
* Merge pull request `#661 <https://github.com/tier4/scenario_simulator_v2/issues/661>`_ from tier4/feature/lane_change_trajectory_shape
* Merge pull request `#660 <https://github.com/tier4/scenario_simulator_v2/issues/660>`_ from tier4/feature/traffic_simulator/vehicle_model
* Merge pull request `#654 <https://github.com/tier4/scenario_simulator_v2/issues/654>`_ from tier4/feature/request_relative_speed_change
* apply reformat
* remove debug line and modify scenario
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* Add new VehicleModelType `DELAY_STEER_VEL`
* enable change lane
* add debug line
* add namespace
* move to .cpp
* add << operator override
* change size_t to uint8_t
* fix problems in always return boost::none
* enable use self entity as reference
* add linear lanechange scenario
* merge fix/galactic_build
* fix compile error
* enable generate linear trajectory
* modify argument type
* change argument type
* change to private
* fix compile error
* add void requestLaneChange(const traffic_simulator::lane_change::Parameter &)
* add copy constructor
* add copy constructor
* add constructor
* rename to_lanelet_id to lane_change_parameters
* add Parameter struct
* add constructor
* add data types for constraint and trajectory
* add Lane change data types
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* remove glog functions
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* update bounding box size while get status
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* remove debug lines
* set other status first
* update requestSpeedChange logic
* add new test case
* enable calculate relative target speed
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* add getAbsoluteValue function in RelativeTargetSpeed class
* add setTargetSpeed(const RelativeTargetSpeed & target_speed, bool continuous) function to the each entity
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into matsuura/feature/add-time-to-panel
* pull master
* merge master
* Merge tier4:master
* fix to build
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge pull request `#655 <https://github.com/tier4/scenario_simulator_v2/issues/655>`_ from tier4/fix/galactic_build
* remove debug line
* remove glog functions
* remove glog from depends
* add glog and use unique_ptr
* add debug line
* add virtual destructor
* Merge pull request `#652 <https://github.com/tier4/scenario_simulator_v2/issues/652>`_ from tier4/feature/traffic_simulator/vehicle_model
* Lipsticks
* Cleanup some switch statements
* Cleanup `EgoEntity::makeSimulationModel`
* Remove message type `VehicleStateCommand` from VehicleModels
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Remove old VehicleModel `sim_model_ideal.[ch]pp`
* Merge pull request `#648 <https://github.com/tier4/scenario_simulator_v2/issues/648>`_ from tier4/feature/request_speed_change
* Merge pull request `#653 <https://github.com/tier4/scenario_simulator_v2/issues/653>`_ from tier4/fix/error_in_driver_model_from_blackboard
* fix typo
* remove recurrent call in setDriverModel function
* Restore AAP's VehicleModels
* apply reformat
* Fix `EgoEntity` to set `GearCommand` to VehicleModel
* add space
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change
* Lipsticks
* set default driver model in pedestrian entity class
* Merge pull request `#650 <https://github.com/tier4/scenario_simulator_v2/issues/650>`_ from tier4/fix/get_driver_model_in_pedestrian
* remove old header file
* Update VehicleModels to match latest `simple_planning_simulator_node`
* fix linter error
* Remove unused enumerations of `VehicleModelType`
* enable pass compile
* add const to the function
* fix way of calling API
* remove old API
* add requestSpeedChange function to the misc object and pedestrian
* change EntityBase::setDriverModel to the pure virtual function
* add data type
* modify test case
* add requestSpeedChange API
* Merge pull request `#646 <https://github.com/tier4/scenario_simulator_v2/issues/646>`_ from tier4/feature/set_acceleration_deceleration
* rename function
* rename function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge pull request `#628 <https://github.com/tier4/scenario_simulator_v2/issues/628>`_ from tier4/feature/avoid_overwrite_acceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* enable pass colcon test
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* add getDriverModel function in egoEntity class
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* add setAcceleration/Deceleration function to the API class
* add setAcceleration/Develeration to the vehicle entity class
* add setAcceleration/Deceleration function to the entity manager class
* add setAcceleration and setDeceleration to the base class
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge pull request `#640 <https://github.com/tier4/scenario_simulator_v2/issues/640>`_ from RobotecAI/fix/multi-lane-traffic-light-stopline-search
* multiple lane traffic light stopline search fix
* Merge pull request `#637 <https://github.com/tier4/scenario_simulator_v2/issues/637>`_ from tier4/feature/pass_goal_poses_to_the_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Merge pull request `#633 <https://github.com/tier4/scenario_simulator_v2/issues/633>`_ from tier4/feature/transform_point
* remove header
* add getGoalPosesInWorldFrame();
* Merge branch 'feature/transform_point' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* use const &
* Merge branch 'feature/transform_point' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* add transformPoint function
* move transform points to the transform.cpp and hpp
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* add key
* add getGoalPoses function to the plugin
* Break indentation (due to ament_clang_format)
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Remove `autoware_auto_msgs` from dependency
* Set default `architecture_type` to `tier4/proposal`
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Update `EntityManager` to select `TrafficLightManager` message type
* Update `TrafficLight(Arrow|Color)` to ROS2 message type conversion
* Update `TrafficLightManager` publisher to be parameterizable
* Merge pull request `#622 <https://github.com/tier4/scenario_simulator_v2/issues/622>`_ from tier4/fix-pointcloud-topic
* fix topic name of pointcloud
* Update `TrafficLightManager` to create publishers by itself
* Update class `SensorSimulation` to choice topic name and type based on Autoware's architecture type
* Add new virtual class `DetectionSensorBase`
* Update `API::attachDetectionSensor` to detect Autoware architecture
* Update `API::attachLidarSensor` to detect Autoware architecture
* Remove `autoware_auto_control_msgs.proto`
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Restore virtual function `EntityBase::getVehicleCommand`
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* Comment-out some tests and Remove protobuf type `GearCommand`, `GearReport`
* change no_ground pointcloud topic name
* Update DetectionSensor to use `autoware_auto_perception_msgs`
* Remove member function `getVehicleCommand` from Vehicle type entity
* use auto_msgs for traffic lights
* remove autoware_auto_msgs dependency
* some changes to run psim with autoware_universe
* Update some packages to use `tier4/autoware_auto_msgs`
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge pull request `#630 <https://github.com/tier4/scenario_simulator_v2/issues/630>`_ from tier4/feature/add_ideal_accel_model
* Merge commit 'ce08abe39ed83d7ec0630560d293187fbcf08b5e' into feature/add_ideal_accel_model
* Merge pull request `#631 <https://github.com/tier4/scenario_simulator_v2/issues/631>`_ from tier4/fix/set_driver_model
* Merge pull request `#619 <https://github.com/tier4/scenario_simulator_v2/issues/619>`_ from RobotecAI/AJD-254-simple_abstract_scenario_for_simple_random_testing
* disable throw errors while calling setDriverModel error
* add IDEAL_ACCEL model
* Merge remote-tracking branch 'tier/master' into AJD-254-simple_abstract_scenario_for_simple_random_testing
* Merge pull request `#626 <https://github.com/tier4/scenario_simulator_v2/issues/626>`_ from tier4/feature/get_driver_model
* add getDriverModel function
* fix compile errors
* move functions into .cpp file
* add getter setter to the base class
* Merge pull request `#623 <https://github.com/tier4/scenario_simulator_v2/issues/623>`_ from RobotecAI/fix/traffic_light_lookup
* traffic lights to stop line lookup fix
* Merge pull request `#621 <https://github.com/tier4/scenario_simulator_v2/issues/621>`_ from tier4/fix/empty_trafic_light
* publish empty message
* change topic name
* Merge pull request `#618 <https://github.com/tier4/scenario_simulator_v2/issues/618>`_ from tier4/fix/remove_lanechange_route
* random test runner
* set withLaneChange parameter as false
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge pull request `#612 <https://github.com/tier4/scenario_simulator_v2/issues/612>`_ from tier4/feature/remove_newton_method_from_get_s_value
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* use autoscale option
* add getNearbyLaneletIds function
* consider bounding box if possible
* remove default argument
* modify test cases
* add bounding box to the argument
* remove debug line
* enable consider edge case (tx and ty sometimes inf)
* simplify get s value algorithum
* add matchToLane function
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Merge branch 'feature/lane_matching' into feature/remove_newton_method_from_get_s_value
* remove newton method
* return boost::none when the value under 0 or over 1
* Revert "Merge pull request `#603 <https://github.com/tier4/scenario_simulator_v2/issues/603>`_ from tier4/fix/get_s_value"
* fix compile error
* add matchToLane function
* add lanelet2_matching package to the external directory
* Merge pull request `#609 <https://github.com/tier4/scenario_simulator_v2/issues/609>`_ from tier4/feature/load_plugin_in_spawn_api
* put values into TODO
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, dai0912th, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge pull request `#603 <https://github.com/tier4/scenario_simulator_v2/issues/603>`_ from tier4/fix/get_s_value
* apply reformat
* use size_t in catmull rom spline class
* use size_t in Hermite Curve class
* fix torelance in test cases
* use multiple initial value
* remove torelance check
* modify parameters
* fix logics if the s value unders 0 and overs 1
* Merge pull request `#597 <https://github.com/tier4/scenario_simulator_v2/issues/597>`_ from tier4/refactor/traffic_simulator/spawning
* Fix function name to lowerCamelCase from snake_case
* Lipsticks
* Replace flag `is_ego` to string typed plugin name
* Update predefined plugin names to use construnst on first use idiom
* Merge branch 'master' into feature/interpreter/catalog
* Add constexpr variable `default_behavior` as entity types member
* Cleanup `API::spawn` for Misc type entity
* Cleanup `API::spawn` for Pedestrian type entity
* Cleanup `API::spawn` for Vehicle type entity
* Update `API::spawn` argument order
* Remove meaningless argument `is_ego` from some `spawn` overloads
* Update `API::spawn` to not to apply `setEntityStatus` to rest arguments
* Update `AddEntityAction::operator ()` to use `TeleportAction::teleport`
* fix typo
* fix calcuration method of normal vector
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: Masaya Kataoka, MasayaKataoka, Yutaro Matsuura, kyabe2718, yamacir-kit

0.5.6 (2021-10-28)
------------------
* Merge pull request `#586 <https://github.com/tier4/scenario_simulator_v2/issues/586>`_ from tier4/fix/get_longitudinal_distance
* fix way of calculating longitudinal distance
* Merge pull request `#585 <https://github.com/tier4/scenario_simulator_v2/issues/585>`_ from tier4/feature/get_nearby_lanelet
* Merge pull request `#584 <https://github.com/tier4/scenario_simulator_v2/issues/584>`_ from tier4/fix/get_closet_lanelet_id
* fix typo
* add s in end
* remove unused orientation
* enable get nearby lanelet
* fix typo
* enable pass plugin name via constructor
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#573 <https://github.com/tier4/scenario_simulator_v2/issues/573>`_ from tier4/feature/behavior_debug_marker
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/galactic_docker_image
* Merge pull request `#575 <https://github.com/tier4/scenario_simulator_v2/issues/575>`_ from tier4/fix/typo
* fix compile errors
* fix typo detected from https://github.com/tier4/scenario_simulator_v2/runs/3923309766?check_suite_focus=true
* rename function
* set initial value in entity constructor
* enable get debug marker from blackboard
* add debug marker setter/getter
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#570 <https://github.com/tier4/scenario_simulator_v2/issues/570>`_ from tier4/feature/cleanup_logger
* change message type name
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* use const & in getCurrentAction function
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* enable publish debug marker
* add appenDebugMarker function
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge pull request `#568 <https://github.com/tier4/scenario_simulator_v2/issues/568>`_ from tier4/feature/clanup_macro_and_blackboard
* remove transition step from setup logger function
* enable pass logger
* sort lines asending
* remove blackboard and modify macro
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#566 <https://github.com/tier4/scenario_simulator_v2/issues/566>`_ from tier4/feature/behavior_plugin
* link stdc++fs
* comment out unused test cases
* fix depends and LICENSE
* modify install line
* add boost to the depends
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* apply reformat
* remove destructor
* use destructor
* add onDespawn function
* remove debug commands
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* remove constructor and add configure function
* enable pass compile
* add debug lines
* fix plugin macro
* enable load plugin
* update config directory
* enable pass compile errors
* remove const
* add BehaviorTreePlugin class
* use base class
* add DEFINE_GETTER_SETTER macro
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* add setter functions
* enable pass compile in traffic_simulator
* define setter/getter
* use shared ptr
* add base class
* add BlackBoard class
* change include path
* change include guard
* move behavior source codes from traffic_simulator to behavior_tree_plugin
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------
* Merge pull request `#562 <https://github.com/tier4/scenario_simulator_v2/issues/562>`_ from tier4/fix/depends_in_rviz
* fix rviz path and package dependency
* Contributors: Masaya Kataoka, MasayaKataoka

0.5.4 (2021-10-13)
------------------
* Merge pull request `#557 <https://github.com/tier4/scenario_simulator_v2/issues/557>`_ from tier4/revert/pr_544
* Revert "Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status"
* Merge pull request `#554 <https://github.com/tier4/scenario_simulator_v2/issues/554>`_ from tier4/feature/autoware/upper-bound-velocity
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Fix Autoware's default upper bound speed to double max from zero
* Add new member function `setUpperBoundSpeed`
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------
* Merge pull request `#549 <https://github.com/tier4/scenario_simulator_v2/issues/549>`_ from tier4/fix/traffic_simulator/autoware
* Fix `EgoEntity::setStatus` to call `VehicleEntity::setStatus`
* Lipsticks
* Merge pull request `#548 <https://github.com/tier4/scenario_simulator_v2/issues/548>`_ from prybicki/patch-6
* Fix SIGABRT due to accessing uninitialized optional
* Contributors: Peter Rybicki, Tatsuya Yamasaki, yamacir-kit

0.5.2 (2021-10-06)
------------------
* Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status
* initialize standstill duration for each entity
* apply reformat
* remove boost::none check
* add spawnEntity function
* remove boost::none from getStandstillDuration function
* remove boost none in each metrics
* enable check entity exists
* move rviz file and configure depends
* add API::
* add spawn function
* remove spawn function without status
* remove unused depend
* use template
* use API::setEntityStatus function
* enable pass compile
* add doxygen comments
* add name argument
* add comment
* remove unused bool return value
* remove boost::none status in traffic_simulator
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#536 <https://github.com/tier4/scenario_simulator_v2/issues/536>`_ from tier4/fix/get_longitudinal_distance
* check target entity is assing to the lane or not
* udpate Release note
* add bool EntityManager::laneMatchingSucceed(const std::string & name)
* Merge pull request `#533 <https://github.com/tier4/scenario_simulator_v2/issues/533>`_ from tier4/feature/interpreter/distance-condition
* Update `getLongitudinalDistance` to support overload for `LaneletPose`
* Merge pull request `#530 <https://github.com/tier4/scenario_simulator_v2/issues/530>`_ from RobotecAI/traffic_lights
* Typos fix
* Clang formatting and conversions test for traffic light
* ZMQ api for traffic lights
* Traffic lights wip
* Merge branch 'master' into fix/clean_directory_behavior
* Merge branch 'master' into rename_AA_launch_package
* Merge pull request `#491 <https://github.com/tier4/scenario_simulator_v2/issues/491>`_ from tier4/feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into fix/interpreter/add-entity-action
* Merge pull request `#511 <https://github.com/tier4/scenario_simulator_v2/issues/511>`_ from tier4/feature/metrics_get_jerk_from_autoware
* trivially fix
* EntityManager has a node as rclcpp::node_interfaces::NodeTopicInterface to erase its type
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge remote-tracking branch 'origin/master' into feature/interpreter/distance-condition
* Merge pull request `#521 <https://github.com/tier4/scenario_simulator_v2/issues/521>`_ from tier4/feature/collision_metric
* update document and fix typo
* Merge pull request `#520 <https://github.com/tier4/scenario_simulator_v2/issues/520>`_ from tier4/feature/standstill_metric
* add test case
* enable specify targets
* enable throw spec violation
* modify cmakelist.txt
* Merge branch 'feature/standstill_metric' of https://github.com/tier4/scenario_simulator_v2 into feature/collision_metric
* add source
* add standstill duration scenario
* add standstill metric
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Merge pull request `#512 <https://github.com/tier4/scenario_simulator_v2/issues/512>`_ from tier4/feature/test_entity
* apply reformat
* add acquire position test cases
* add a subscription to get jerk
* add test case for set status and update timestamp
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Cleanup member function `EgoEntity::getCurrentAction`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Support new UserDefinedValueCondition `<ENTITY-NAME>.currentState`
* Support new member function `API::getCurrentAction`
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Jaroszek, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge pull request `#507 <https://github.com/tier4/scenario_simulator_v2/issues/507>`_ from tier4/feature/add_scenario
* update lane assing logic for pedestrian
* split function
* apply reformat
* enable get lanelet pose while walk straight action
* Merge pull request `#505 <https://github.com/tier4/scenario_simulator_v2/issues/505>`_ from tier4/feature/test_helper
* modify line
* add test case for lidar sensor
* add test case for constructing action status
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* gix some typo
* remove dists
* fix some typo
* fix filename
* use foo/bar/baz
* fix typo of Bounding
* fix typo of polynomial
* fix typo of cache
* fix typo of Valuet
* fix compile error
* change dist to distance
* fix typo of tolerance
* add test case for helper function
* Merge pull request `#501 <https://github.com/tier4/scenario_simulator_v2/issues/501>`_ from tier4/feature/add_test_traffic_light
* Merge pull request `#500 <https://github.com/tier4/scenario_simulator_v2/issues/500>`_ from tier4/fix/offset_calculation
* add test cases for >> operator
* add ss = std::stringstream(); lines
* add test case for operator <<
* Merge branch 'master' into fix/offset_calculation
* add // LCOV_EXCL_LINE
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* remove std::hypot(x,y,z)
* Merge pull request `#458 <https://github.com/tier4/scenario_simulator_v2/issues/458>`_ from Utaro-M/add-goalpose
* add subtraction
* Merge pull request `#486 <https://github.com/tier4/scenario_simulator_v2/issues/486>`_ from prybicki/patch-5
* fix toMapPose function
* add test cases
* add operator override
* add getSize function
* Fix bad formatting
* add linear algebra.cpp
* Merge pull request `#498 <https://github.com/tier4/scenario_simulator_v2/issues/498>`_ from tier4/feature/remove_unused_codes_in_entity
* remove pedestrian_parameters.hpp and vehicle_parameters.hpp
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_unused_codes_in_entity
* fix typo
* Merge branch 'master' into add-goalpose
* Merge pull request `#492 <https://github.com/tier4/scenario_simulator_v2/issues/492>`_ from tier4/feature/add_traffic_light_test
* use static cast in std::accumulate function
* use reference
* remove unused function
* remove unused function
* add test cases for getArrow function
* add test line
* remove std::accumulate because of overflow
* fix update logic in traffic light phase class
* add update line in test case
* add setColorPhase test cases
* fix error
* fix values
* add getColorAndArrowPosition test cases
* add test cases for getArrowPosition
* add expect macro
* add set arrow function
* use foreach
* add test cases for setColor function
* remove test case file
* add new test case source
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge pull request `#489 <https://github.com/tier4/scenario_simulator_v2/issues/489>`_ from tier4/feature/test_traffic_light
* use std::find instead of std::count_if
* add test case for getIDs
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* add new test case file
* Merge pull request `#487 <https://github.com/tier4/scenario_simulator_v2/issues/487>`_ from tier4/feature/get_longituninal_distance_behind
* add test case for arrow and NONE type
* enable get distance from behind entity
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Merge pull request `#485 <https://github.com/tier4/scenario_simulator_v2/issues/485>`_ from tier4/feature/test_simulation_interface
* add test case for makeLampState function
* Update `EgoEntity` to default construct `Autoware` if `launch_autoware == false`
* Merge branch 'master' into add-goalpose
* fix typo
* Set name in the proto request for non-ego vehicles
* Support new option `initialize_duration`
* Update class `EgoEntity` to don't instantiate class `Autoware` if `launch_autoware == false`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Rename option `architecture-type` to `architecture_type`
* Remove unreachable return in API::spawn (`#480 <https://github.com/tier4/scenario_simulator_v2/issues/480>`_)
* Set correct entity names in proto messages (`#481 <https://github.com/tier4/scenario_simulator_v2/issues/481>`_)
* Feature/request acuire position in world coordinate (`#439 <https://github.com/tier4/scenario_simulator_v2/issues/439>`_)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* fix typo
* add goalpose arrow
* add getGoalposes()
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Peter Rybicki, Piotr Rybicki, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Fix/offset calculation in lane coordinte (`#476 <https://github.com/tier4/scenario_simulator_v2/issues/476>`_)
* Merge pull request `#475 <https://github.com/tier4/scenario_simulator_v2/issues/475>`_ from tier4/feature/add_math_test
* Merge pull request `#437 <https://github.com/tier4/scenario_simulator_v2/issues/437>`_ from RobotecAI/issue/AJD-237-remove_autoware_compilation_flag
* add getSvalue test
* modify test cases in getTrajectory
* add test case for get trajectory function
* add test case for search backwards
* fix compile errors
* remove using
* Merge remote-tracking branch 'origin/master' into fix/interpreter/misc
* Feature/metrics test (`#469 <https://github.com/tier4/scenario_simulator_v2/issues/469>`_)
* Feature/remove unused constructor (`#465 <https://github.com/tier4/scenario_simulator_v2/issues/465>`_)
* Merge pull request `#464 <https://github.com/tier4/scenario_simulator_v2/issues/464>`_ from tier4/feature/add_math_test
* fix build and formatting after rebase
* review changes
* apply clang-format
* cleanup
* AAP acceleration fix
* make Autoware switch based on autoware_type parameter
* AAP builds
* first version that builds without flag and works with AA on autoware-simple scenario
* move Autoware differences from ego_entity to concealer
* WIP: move Autoware differences from ego_entity to concealer
* remove unused temporal value
* add collision test case
* add subdirectory
* enable matrix test
* add test cases for solve quadratic equation
* fix quadratic function ans
* add quadratic function case
* add LinerFunction case
* add range test
* add LinerFunction
* update uuid test case
* change test name
* remove math.cpp
* add uuid case
* modify testcase name
* add bounding box test case
* add subdirectory
* add catmull rom spline test cases
* remove unused code
* remove unused test case
* add fixture class
* add hermite curve test cases
* change test case name
* split test cases
* add test cases for getCollisionPositionIn2D function
* add test case for auto scale
* initialize current_time\_ with negative value so setTargetSpped and setTargetVelocity are deterministic (`#462 <https://github.com/tier4/scenario_simulator_v2/issues/462>`_)
* Feature/move backward action (`#461 <https://github.com/tier4/scenario_simulator_v2/issues/461>`_)
* Merge pull request `#457 <https://github.com/tier4/scenario_simulator_v2/issues/457>`_ from tier4/feature/math_test
* fix typo
* add // LCOV_EXCL_LINE
* chceck
* add test cases
* add  // LCOV_EXCL_LINE
* add  // LCOV_EXCL_LINE
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* add // LCOV_EXCL_LINE
* add // LCOV_EXCL_LINE
* remove unused message
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* add test case in maximum curventure
* add get2DMinMaxCurventureValue function
* adding test cases
* remove unused message
* add error test
* modify error message
* remove unused constractor
* remove unused line
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge pull request `#451 <https://github.com/tier4/scenario_simulator_v2/issues/451>`_ from tier4/feature/out-of-range-metric
* Merge pull request `#452 <https://github.com/tier4/scenario_simulator_v2/issues/452>`_ from tier4/fix/stop_at_crossing_entity_behavior
* remove unused line
* remove unused line
* add test scenario
* fix problems in stop_at_crossing_entity action
* Merge pull request `#450 <https://github.com/tier4/scenario_simulator_v2/issues/450>`_ from tier4/fix/phase_control
* fix
* add OutOfRangeMetric when vehicle is spawned
* enable filter result
* remove const
* fix problems in TrafficLightManager::update() function
* Merge pull request `#448 <https://github.com/tier4/scenario_simulator_v2/issues/448>`_ from tier4/feature/add_cpp_scenarios
* add call thread::join in destructor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge pull request `#446 <https://github.com/tier4/scenario_simulator_v2/issues/446>`_ from tier4/feature/out-of-range-metric
* fix exception message
* add out_of_range_metric.cpp to CMakeLists.txt
* add OutOfRangeMetric
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* add linear velocity to MomentaryStopMetric
* add metrics scenario
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge pull request `#432 <https://github.com/tier4/scenario_simulator_v2/issues/432>`_ from tier4/fix/suppress_warnings
* remove boost none in getFrontEntityName logic
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/suppress_warnings
* Merge pull request `#440 <https://github.com/tier4/scenario_simulator_v2/issues/440>`_ from tier4/fix/follow_front_entity_behavior
* leave @note line
* use const &
* use const &
* remove unused temporal value
* fix torerance
* remove debug lines
* fix cubic function
* enable check collision
* Merge branch 'master' into namespace
* change default value
* add close_start_end option
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#434 <https://github.com/tier4/scenario_simulator_v2/issues/434>`_ from tier4/feature/remove_get_s_valueIn_route
* remove unused functions
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#433 <https://github.com/tier4/scenario_simulator_v2/issues/433>`_ from tier4/feature/yeild_to_merging_entity
* apply reformat
* remove debug lines
* add getEntityStatus function
* apply reformat
* get polygon to the other status
* fix to old version get longitudinal distance function
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/lane_change_npc_distance_in_lane_coordinate
* fix no return
* fix unused
* fix uninitialized
* fix rclcpp::Duration deprecated
* add Werror
* change quadratic function
* Merge remote-tracking branch 'origin/master' into namespace
* check error
* fix s value
* check tx and ty value
* fix othre conditions
* fix collision solver for cubic function
* enable calculate distance to front entity by spline
* enable get distance between polygon
* remove unused result
* remove debug lines
* use getConflictingEntityStatusOnRoute in getConflictingEntityStatus function
* enable get conflicting vehicle entity in foundConflictingEntity function
* remove unused function
* add std::vector<std::int64_t> HdMapUtils::getConflictingLaneIds
* Merge pull request `#429 <https://github.com/tier4/scenario_simulator_v2/issues/429>`_ from tier4/feature/add_cpp_scenarios
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* set initial value for current_s\_
* fix argument name
* fix compile errors
* Merge branch 'fix/add_include_crosswalk_option' of github.com:tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* Contributors: Hiroki OTA, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------
* Merge pull request `#425 <https://github.com/tier4/scenario_simulator_v2/issues/425>`_ from tier4/fix/add_include_crosswalk_option
* Merge pull request `#408 <https://github.com/tier4/scenario_simulator_v2/issues/408>`_ from RobotecAI/issue/AJD-239-non_hardcoded_map
* apply reformat
* use calculateEntityStatusUpdated function
* kashiwanoha_map package map files added
* Contributors: Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki

0.4.1 (2021-07-30)
------------------
* Merge pull request `#421 <https://github.com/tier4/scenario_simulator_v2/issues/421>`_ from tier4/fix/npc_target_speed_in_follow_front_entity_action
* fix velocity planner logic
* apply reformat
* Merge pull request `#419 <https://github.com/tier4/scenario_simulator_v2/issues/419>`_ from tier4/feature/rename_moc_to_mock
* enable set waypoints
* fix login in follow front entity action
* remove launch install line
* remove moc directory
* move to mock package
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Contributors: Masaya Kataoka, yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#407 <https://github.com/tier4/scenario_simulator_v2/issues/407>`_ from tier4/feature/galactic_support
* apply reformat
* fix usage of declare_parameter function
* check boost::none in update function
* remove undeclare line
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Move flag 'autoware_initialized' into class 'Autoware'
* Update EgoEntity to occupy one Autoware each
* Remove debug codes from EgoEntity
* Remove member function 'getMapPath'
* Add member function 'get*MapFile' to struct Configuration
* Update class Configuration to assert .osm file existence
* Update class Configuration to assert given map_path
* Add data member 'rviz_config_path' to struct Configuration
* Update EgoEntity's constructor to receive Configuration
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* Lipsticks
* Remove some unused data members
* Merge pull request `#403 <https://github.com/tier4/scenario_simulator_v2/issues/403>`_ from tier4/feature/target_speed_planner
* Update EntityManager's constructor to receive Configuration
* add const
* enable use target speed planner in pedestrian entity
* Fix lanelet projector
* enable use target_speed_planner in vehicle entity
* Move class 'Configuration' into new header 'configuration.hpp'
* Update MetricsManager to receive boost::filesystem::path
* Cleanup
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* add new file
* Update HDMapUtils's constructor to receive boost::filesystem::path
* remove value
* Merge pull request `#400 <https://github.com/tier4/scenario_simulator_v2/issues/400>`_ from tier4/feature/remove_unused_member_value_in_entity
* Merge pull request `#401 <https://github.com/tier4/scenario_simulator_v2/issues/401>`_ from tier4/fix/typo
* Remove some duplicated API's data members
* remove getCurrentAction and replace to tickOnce(current_time, step_time);
* fix typo of calculate
* change return type to void
* remove action status from pedestrian entity
* Add new struct 'Configuration' for class 'API'
* fix compile error
* remove from member
* remove action_status from vehicle entity
* Merge remote-tracking branch 'origin/master' into fix/interpreter/acquire-position-action
* Merge pull request `#390 <https://github.com/tier4/scenario_simulator_v2/issues/390>`_ from tier4/feature/interpreter/traffic-signal-controller-condition
* Fix syntax Event and ManeuverGroup to be able to restart elements
* Update TrafficLightManager to store TrafficLight objects directly
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Update enumeration 'Arrow'
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge pull request `#386 <https://github.com/tier4/scenario_simulator_v2/issues/386>`_ from tier4/feature/interpreter/misc-object
* Fix rviz config paths
* Merge pull request `#387 <https://github.com/tier4/scenario_simulator_v2/issues/387>`_ from tier4/fix/delete_whole_route_when_empty
* apply reformat
* fix route planner logic
* Merge remote-tracking branch 'origin/master' into feature/interpreter/test-scenario
* Merge pull request `#384 <https://github.com/tier4/scenario_simulator_v2/issues/384>`_ from tier4/feature/interpreter/assign-route-action-with-world-position
* Merge remote-tracking branch 'origin/master' into feature/interpreter/assign-route-action-with-world-position
* Merge pull request `#328 <https://github.com/tier4/scenario_simulator_v2/issues/328>`_ from RobotecAI/pjaroszek/map_and_planning
* Lipsticks
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* clang formatting
* enable send SpawnMiscObjectEntityRequest to the sensor simulator
* update mock
* add despawn line
* specify entity type
* fix debug message
* fix compile errors
* adapt formatting
* rebase adjustments
* add constructor
* add MiscObjectParameters message type
* Merge branch 'master' into traffic_signal_actions
* build with AUTOWARE_AUTO flag defined instead of AUTOWARE_ARCHITECTURE_PROPOSAL
* Merge pull request `#379 <https://github.com/tier4/scenario_simulator_v2/issues/379>`_ from tier4/fix/get_waypoints_error_message
* fix debug line
* apply reformat
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/get_waypoints_error_message
* chenge error message
* Merge pull request `#378 <https://github.com/tier4/scenario_simulator_v2/issues/378>`_ from tier4/feature/ego-entity/acuquire-position-action
* Update EgoEntity to be able to request AcquirePositionAction multiple times
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.2.0 (2021-06-24)
------------------
* Merge pull request `#372 <https://github.com/tier4/scenario_simulator_v2/issues/372>`_ from tier4/fix/lane_change_parameter
* add New lines at the end of the file
* apply reformat
* input optional arguments
* update lane change logic
* move getRelativePose function to the math directory
* enable set maximum_curvature_threshold parameter
* Merge pull request `#357 <https://github.com/tier4/scenario_simulator_v2/issues/357>`_ from tier4/feature/send_ego_command
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* enable use getEntityStatusBeforeUpdate function in EntityManager class
* add getEntityStatusBeforeUpdate() function
* fix problems while getting ego status
* remove warnings and add getEgoName function to the API class
* add getEgoName function to the entity manager class
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge pull request `#363 <https://github.com/tier4/scenario_simulator_v2/issues/363>`_ from tier4/fix/throw_errors_when_set_target_speed_after_ego_starts
* apply clanf-format
* enable throw semantic error for setTargetSpeed and setEntityStatus after starting simulation
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'feature/interpreter/context' of github.com:tier4/scenario_simulator_v2 into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge branch 'master' into relative_target_speed
* Revert "add RelativeTargetSpeed support to interpreter"
* add RelativeTargetSpeed support to interpreter
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge pull request `#355 <https://github.com/tier4/scenario_simulator_v2/issues/355>`_ from tier4/feature/get_vehicle_cmd
* add getVehicleCommand class to the API class
* add const autoware_vehicle_msgs::msg::VehicleCommand getVehicleCommand(); function to the EntityManager class
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/get_vehicle_cmd
* add getVehicleCommand function to the ego entity class
* Merge pull request `#354 <https://github.com/tier4/scenario_simulator_v2/issues/354>`_ from tier4/fix/typos-misc
* Apply clang-format
* Fix typos in docs / mock / simulation/ test_runner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into doc/update_image
* Merge pull request `#351 <https://github.com/tier4/scenario_simulator_v2/issues/351>`_ from tier4/fix/traffic-simulator/simulation-model-2
* Replace some identifiers spellcheck reported as issue
* Replace temporary excptions
* Update EgoEntity::setTargetSpeed to consider parameter 'vehicle_model_type'
* Support simulation model 'SimModelIdealSteer'
* Update EgoEntity to use model specified by parameter 'vehicle_model_type'
* Update EgoEntity to read parameter 'vehicle_model_type'
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/replay.launch
* Merge pull request `#345 <https://github.com/tier4/scenario_simulator_v2/issues/345>`_ from tier4/fix/traffic-simulator/simulation-model
* Fix control inputs
* Add some debug prints
* Update EgoEntity's simulation model to use SimModelTimeDelaySteerAccel
* Merge pull request `#343 <https://github.com/tier4/scenario_simulator_v2/issues/343>`_ from tier4/fix/traffic-simulator/vehicle-parameter
* Fix API to pass step-time to EgoEntity's constructor
* Add debug prints
* Remove debug print
* Update launch file to receive LaunchContext
* Merge pull request `#338 <https://github.com/tier4/scenario_simulator_v2/issues/338>`_ from tier4/feature/interpreter/vehicle-description
* Update EgoEntity to use precise simulation model parameters
* Update scenario_test_runner.launch.py to load Autoware parameter
* Merge pull request `#336 <https://github.com/tier4/scenario_simulator_v2/issues/336>`_ from tier4/feature/speed_up_npc_logic
* add meanings  of sampling resolution
* fixd problems described in review
* remove verbose true
* remove unused lines
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* apply reformat
* Fix typos and grammars in docs and comments
* apply reformat
* add LaneletLengthChache
* filter other entity status by cartesian distance
* comment out multithread section
* commend out stop watch
* calculate waypoints only one time in follow lane action
* add center points cache
* fix typo
* apply reformat
* use future and async launch
* remove warning
* add parallel util
* use openmp
* add omp to the depends
* fix typo
* update branch
* apply reformat
* enable cache length
* apply reformat
* add route cache
* Merge pull request `#316 <https://github.com/tier4/scenario_simulator_v2/issues/316>`_ from tier4/fix/hold_stream
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into fix/hold_stream
* apply reformat
* hold ostrem in manager class
* change default value
* apply reformat
* add file_output_every_frame option (default = false)
* Merge pull request `#315 <https://github.com/tier4/scenario_simulator_v2/issues/315>`_ from tier4/feature/use_ros_clock
* enable use raw ros timestamp
* Merge pull request `#314 <https://github.com/tier4/scenario_simulator_v2/issues/314>`_ from tier4/fix/sensor_timestamp
* apply reformat
* Merge pull request `#306 <https://github.com/tier4/scenario_simulator_v2/issues/306>`_ from tier4/feature/use_common_exception
* Remove trailing semicolon from macro definition
* fix problems in https://github.com/tier4/scenario_simulator_v2/pull/306#discussion_r634055658
* fix typo
* add spec violation error
* remove BehaviorTree exception
* remove CALCURATION_ERROR
* remove SimulationClockError
* remove Execution Error
* remove some exception
* remove exception.hpp
* remove hdmap error
* use common exception
* remove traffic_simulator::SimulationRuntimeError
* modify macro
* Merge pull request `#304 <https://github.com/tier4/scenario_simulator_v2/issues/304>`_ from tier4/feature/synchronize_clock
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronize_clock
* enable send clock
* Merge pull request `#302 <https://github.com/tier4/scenario_simulator_v2/issues/302>`_ from tier4/feature/error-handling-2
* Merge pull request `#301 <https://github.com/tier4/scenario_simulator_v2/issues/301>`_ from tier4/feature/publish_clock
* add licence
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge pull request `#297 <https://github.com/tier4/scenario_simulator_v2/issues/297>`_ from tier4/feature/error-handling
* use from_seconds function
* enable publish clock
* add publisher
* Update Interpreter to destruct simulator on deactivation phase
* remove unused params
* add clock class to the api
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* add SimulationClock class
* Merge remote-tracking branch 'origin/master' into feature/error-handling
* Update EgoEntity to show current-time
* Contributors: Kazuki Miyahara, Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#295 <https://github.com/tier4/scenario_simulator_v2/issues/295>`_ from tier4/fix/python_format
  reformat by black
* reformat by black
* Merge pull request `#294 <https://github.com/tier4/scenario_simulator_v2/issues/294>`_ from tier4/feature/support-autoware.iv-0.11.2
  Feature/support autoware.iv 0.11.2
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* Update EgoEntity::getCurrentAction to return non-empty string
* remove flake8 check
* add new line for the block
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* Update scenario_test_runner.launch.py to receive sensor and vehicle model
* Merge pull request `#287 <https://github.com/tier4/scenario_simulator_v2/issues/287>`_ from tier4/feature/remove-dummy-perception-publisher
  Feature/remove dummy perception publisher
* Rename package 'awapi_accessor' to 'concealer'
* Update Autoware::ready to rethrow exception if there is thrown exception
* Merge pull request `#281 <https://github.com/tier4/scenario_simulator_v2/issues/281>`_ from tier4/feature/asynchronous-autoware-initialization
  Feature/asynchronous autoware initialization
* Update Storyboard to call engage if Autoware is ready (= WaitingForEngage)
* Lipsticks
* Cleanup EntityManager
* Update EntityManager to hold shared_ptrs as const
* Sort member functions of EntityManager
* Add member function EgoEntity::ready
* Update Autoware::engage to be synchronous
* Sort member functions of EgoEntity
* Add new virtual function EntityBase::getEntityTypename
* Update EntityBase to define default implementation of requestWalkStraight
* Sort member functions
* Generalize member function 'Autoware::plan's argument
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/update_contact_information
* Rename member function 'drive' to 'plan'
* Lipsticks
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-0.11.1
* Merge pull request `#276 <https://github.com/tier4/scenario_simulator_v2/issues/276>`_ from tier4/feature/autoware-high-level-api
  Feature/autoware high level api
* Move core procedures of requestAcquirePosition into class 'Autoware'
* Merge pull request `#277 <https://github.com/tier4/scenario_simulator_v2/issues/277>`_ from tier4/doc/docker
  Doc/docker
* Fix some EntityBase's member functions to be virtual
* Fix some EntityBase class member functions
* Update class 'Autoware' to update vehicle informations continuously
* Update EgoEntity to hold class 'Autoware' as non-pointer
* add virtual and override
* add debug lines
* remove warnings
* Update promise to be non-pointer data member
* Update EgoEntity to be uncopyable
* Lipsticks
* Move Autoware process control into class 'Autoware'
* Move Autoware initialization into class 'Autoware' from 'EgoEntity'
* Move member function 'updateAutoware' into AutowareAPI
* Move EgoEntity's member functions 'waitForAutowareStateToBe...' into awapi
* Merge branch 'feature/support-autoware.iv-0.11.1' into feature/autoware-high-level-api
* Merge pull request `#274 <https://github.com/tier4/scenario_simulator_v2/issues/274>`_ from tier4/refactor/cleanup-ego-entity
  Refactor/cleanup ego entity
* Update entity_base::setDriverModel to be virtual
* Fix bug
* Merge branch 'refactor/cleanup-ego-entity' into feature/autoware-high-level-api
* Merge remote-tracking branch 'origin/master' into refactor/cleanup-ego-entity
* Merge pull request `#275 <https://github.com/tier4/scenario_simulator_v2/issues/275>`_ from tier4/feature/init_duration
  Feature/init duration
* return null obstacle when current time while initializing
* fix problems in getting obstacle
* add debug lines
* Rename 'awapi_accessor' to 'autoware'
* enable visualize obstacle
* Lipsticks
* apply reformat
* update mock
* Rename autoware_api::Accessor to awapi::Autoware
* remove at function when I emplace value
* remove boost::any
* Update EgoEntity::launchAutoware to receive map paths
* Convert launch_autoware to member function from closure
* Move EgoEntity::initializeAutoware into ego_entity.cpp
* Convert 'get_parameter' to template function from generic lambda
* Move EgoEntity::requestAcquirePosition into ego_entity.cpp
* Move EgoEntity's destructor into ego_entity.cpp
* enable spawn entity
* Move EgoEntity::EgoEntity into ego_entity.cpp
* Cleanup comments
* remove debug lines
* add init_duration
* Update awapi_accessor to publish '/localization/twist'
* Update EgoEntity to launch Autoware via autoware_launch
* Merge branch 'master' into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#265 <https://github.com/tier4/scenario_simulator_v2/issues/265>`_ from tier4/feature/interpolate_two_center_points
  interpolate center points if the center points are only two points
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpolate_two_center_points
* interpolate center points if the center points are only two points
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#263 <https://github.com/tier4/scenario_simulator_v2/issues/263>`_ from tier4/feature/traffic-signal-sensor
  Feature/traffic signal sensor
* Merge pull request `#264 <https://github.com/tier4/scenario_simulator_v2/issues/264>`_ from tier4/revert/interpolate_two_points
  Revert "enable interpolate two points"
* Revert "enable interpolate two points"
  This reverts commit 7b08f1d0de38e9b31e1d066d5c6ed7faec6758bd.
* enable interpolate two points
* Update traffic signals topic name to use AWAPI
* Update TrafficLightArrow to support conversion to 'LampState' type
* Lipsticks
* Update TrafficLightColor to support conversion to 'LampState'
* Add stream input/output operator to TrafficLight(Arrow|Color)
* Lipsticks
* Rename 'PhaseLength' to 'PhaseDuration'
* Unify some member function definitions into a macro
* Update EntityManager to pass traffic light states publisher to TrafficLightManager
* Merge https://github.com/tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Lipsticks
* Merge pull request `#262 <https://github.com/tier4/scenario_simulator_v2/issues/262>`_ from tier4/feature/interpreter/traffic-signal-controller-2
  Feature/interpreter/traffic signal controller 2
* Add local macro 'RENAME'
* Lipsticks
* Merge branch 'master' into fix/misc-problems
* Merge pull request `#238 <https://github.com/tier4/scenario_simulator_v2/issues/238>`_ from tier4/feature/interpreter/vehicle/base_link-offset
  Remove member function `API::spawn` receives XML strings.
* Remove member function 'API::spawn' receives catalog XML
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* fix launch file
* update namespace
* use clang_format
* apply reformat
* Merge https://github.com/tier4/scenario_simulator.auto into feature/rename_packages
* modify include gurard
* rename simulation_api package
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, yamacir-kit
