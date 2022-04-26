^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package behavior_tree_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.4 (2022-04-26)
------------------
* Merge pull request `#762 <https://github.com/tier4/scenario_simulator_v2/issues/762>`_ from tier4/fix/email
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* fix email address
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge pull request `#740 <https://github.com/tier4/scenario_simulator_v2/issues/740>`_ from tier4/refactor/traffic_simulator/traffic_light_manager
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge pull request `#751 <https://github.com/tier4/scenario_simulator_v2/issues/751>`_ from tier4/feature/behavior_request_enum
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Switch struct `TrafficLight` to experimental version
* change data type
* modify reset request
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_request_enum
* Merge branch 'feature/behavior_request_enum' into feature/reset_acecel_in_request_speed_change
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/job_system
* fix compile errors
* use enum
* Merge pull request `#747 <https://github.com/tier4/scenario_simulator_v2/issues/747>`_ from tier4/fix/calculate_stop_distance
* apply reformat
* fix compile errors
* fix calculateStopDistance function
* Merge branch 'master' into fix/interpreter/interrupt
* Remove member function `TrafficLightManager::get(Arrow|Color)`
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge remote-tracking branch 'origin/master' into feature/interpreter/object-controller
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#708 <https://github.com/tier4/scenario_simulator_v2/issues/708>`_ from RobotecAI/AJD-331-optimization
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#734 <https://github.com/tier4/scenario_simulator_v2/issues/734>`_ from tier4/fix/interpreter/global-action
* Remove unnecessary prints from the interpreter and simulator
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* review changes: common_spline -> reference_trajectory
* Merge branch 'master' into AJD-331-optimization
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* fix build warnings
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* clang format
* remove spline recalculation in calculateObstacle()
* remove spline recalculation in calculateWaypoints()
* calculate and pass common_spline
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge pull request `#714 <https://github.com/tier4/scenario_simulator_v2/issues/714>`_ from tier4/fix/get_longitudinal_distance
  Fix/get longitudinal distance
* change hard coded parameter
* configure parameter
* Merge pull request `#709 <https://github.com/tier4/scenario_simulator_v2/issues/709>`_ from tier4/feature/waypoint_offset
  Feature/waypoint offset
* Merge branch 'feature/waypoint_offset' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* configure state machine
* remove debug line
* add offset in waypoint calculation
* Merge pull request `#703 <https://github.com/tier4/scenario_simulator_v2/issues/703>`_ from tier4/fix/front_entity_detection
  check yaw difference
* use boost::math::constants::half_pi<double>()
* check yaw difference
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Remove `foxy` from GitHub workflows
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#686 <https://github.com/tier4/scenario_simulator_v2/issues/686>`_ from tier4/fix/warp_problem
  Fix/warp problem
* lane matching fails when the offset overs 1
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge branch 'master' into fix/interpreter/lifecycle
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge pull request `#671 <https://github.com/tier4/scenario_simulator_v2/issues/671>`_ from tier4/fix/lane_change_trajectory_shape
* Merge pull request `#670 <https://github.com/tier4/scenario_simulator_v2/issues/670>`_ from tier4/feature/request_speed_change_in_pedestrian
* Merge branch 'feature/request_speed_change_in_pedestrian' of https://github.com/tier4/scenario_simulator_v2 into fix/lane_change_trajectory_shape
* enable set acceleration while walk straight state
* modify trajectory tangent size
* change speed linear in walk straight scenario
* fix problem in passing driver model in pedestrian behavior plugin
* add protected params
* add protected parameters
* Merge remote-tracking branch 'origin/master' into feature/interpreter/speed-action
* Merge pull request `#664 <https://github.com/tier4/scenario_simulator_v2/issues/664>`_ from tier4/feature/lateral_velocity_constraint
* enable change adaptive parameters
* modify scenario
* add TIME constraint
* fix typo
* use switch
* modify test scenario
* split NPC logic by using constraint type
* Merge pull request `#661 <https://github.com/tier4/scenario_simulator_v2/issues/661>`_ from tier4/feature/lane_change_trajectory_shape
* remove debug line
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* sort lines
* add debug line
* move to .cpp
* add << operator override
* merge fix/galactic_build
* modify argument type
* add copy constructor
* add constructor
* rename to_lanelet_id to lane_change_parameters
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into matsuura/feature/add-time-to-panel
* pull master
* merge master
* Merge tier4:master
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge pull request `#655 <https://github.com/tier4/scenario_simulator_v2/issues/655>`_ from tier4/fix/galactic_build
* remove shared ptr
* remove unused member
* add glog and use unique_ptr
* modify TimeStampType in behavior_tree_cpp_v3
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Merge pull request `#637 <https://github.com/tier4/scenario_simulator_v2/issues/637>`_ from tier4/feature/pass_goal_poses_to_the_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* add getGoalPoses function to the plugin
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Update `TrafficLightManager` publisher to be parameterizable
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into AJD-254-simple_abstract_scenario_for_simple_random_testing
* Merge pull request `#624 <https://github.com/tier4/scenario_simulator_v2/issues/624>`_ from tier4/fix/use_deceleration_acceleration_in_driver_model
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/use_deceleration_acceleration_in_driver_model
* Merge pull request `#625 <https://github.com/tier4/scenario_simulator_v2/issues/625>`_ from tier4/fix/ci
* add FOXY definition
* remove hard coded parameters in behavior tree plugin and use accelearaion and deceleration value in traffic_simulator_msgs/msg/DriverModel
* Merge pull request `#616 <https://github.com/tier4/scenario_simulator_v2/issues/616>`_ from tier4/fix/current_action
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* remove current_action\_ member variable from inherit class
* Merge pull request `#612 <https://github.com/tier4/scenario_simulator_v2/issues/612>`_ from tier4/feature/remove_newton_method_from_get_s_value
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* consider bounding box if possible
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* Contributors: kyabe2718

0.5.6 (2021-10-28)
------------------
* Merge pull request `#593 <https://github.com/tier4/scenario_simulator_v2/issues/593>`_ from tier4/fix/use_final_keyword
* Merge pull request `#592 <https://github.com/tier4/scenario_simulator_v2/issues/592>`_ from tier4/fix/version
* Limit overrides by final keyword rather than comments
* update version
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#573 <https://github.com/tier4/scenario_simulator_v2/issues/573>`_ from tier4/feature/behavior_debug_marker
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/galactic_docker_image
* Merge pull request `#575 <https://github.com/tier4/scenario_simulator_v2/issues/575>`_ from tier4/fix/typo
* fix typo detected from https://github.com/tier4/scenario_simulator_v2/runs/3923309766?check_suite_focus=true
* add debug marker setter/getter
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#570 <https://github.com/tier4/scenario_simulator_v2/issues/570>`_ from tier4/feature/cleanup_logger
* remove reference from member variable
* use void(const std::string &) in set_request_function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* use const & in getCurrentAction function
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Merge pull request `#568 <https://github.com/tier4/scenario_simulator_v2/issues/568>`_ from tier4/feature/clanup_macro_and_blackboard
* remove transition step from setup logger function
* remvoe unused lines
* change output format
* remove unused lines
* add logging event
* rename event class
* configure directory
* add transition event class
* enable pass logger
* sort lines asending
* remove blackboard and modify macro
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#566 <https://github.com/tier4/scenario_simulator_v2/issues/566>`_ from tier4/feature/behavior_plugin
* remove unused member variable
* use getRequest() and remove request\_
* enable pass variable
* remove debug lines
* add debug output in mock node
* enable load plugin
* remove ament_cmake_auto
* remove constructor and add configure function
* enable pass compile
* change base class
* fix plugin macro
* enable load plugin
* update config directory
* change base class of vehicle
* enable pass compile errors
* define getters and setters
* remove const
* use base class in pedestrian
* add BehaviorTreePlugin class
* use base class
* rename library
* change include path
* change include guard
* move behavior source codes from traffic_simulator to behavior_tree_plugin
* add behavior_tree_plugin package
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit
