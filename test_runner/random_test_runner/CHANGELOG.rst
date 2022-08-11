^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package random_test_runner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.5 (2022-06-16)
------------------
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* fix(random_test_runner): modify build error in both galactic and humble
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_lane_bound
* Merge pull request `#796 <https://github.com/tier4/scenario_simulator_v2/issues/796>`_ from tier4/refactor/concealer/virtual-functions
* Merge branch 'master' into feature/change_engage_api_name
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Remove member function `API::engage` and `API::ready`
* Merge pull request `#778 <https://github.com/tier4/scenario_simulator_v2/issues/778>`_ from tier4/feature/zmqpp_vendor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Fix Licence
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into feature/zmqpp_vendor
* Merge pull request `#785 <https://github.com/tier4/scenario_simulator_v2/issues/785>`_ from tier4/doc/improve
* Fix broken links
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* modify CMakeLists.txt
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Adam Krawczyk, Daisuke Nishimatsu, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge pull request `#697 <https://github.com/tier4/scenario_simulator_v2/issues/697>`_ from RobotecAI/AJD-345-random_test_runner_with_autoware_universe
* added namespace to nodes in random_test_runner
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/job_system
* Merge pull request `#690 <https://github.com/tier4/scenario_simulator_v2/issues/690>`_ from RobotecAI/AJD-331-make-zmq-client-work-through-network
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' into fix/interpreter/interrupt
* apply clang format
* fix typo
* refactor
* fixes and cleanup
* ugly but working
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* rename data field
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* fix compile error
* fix compile errors in simulator
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* enable pass copile in all packages
* apply clang format
* fix typo
* refactor
* fixes and cleanup
* ugly but working
* change to option 1 (simulator host as a launch parameter)
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge branch 'master' into fix/interpreter/lifecycle
* Contributors: kyabe2718, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge pull request `#670 <https://github.com/tier4/scenario_simulator_v2/issues/670>`_ from tier4/feature/request_speed_change_in_pedestrian
* remove old API
* Merge branch 'feature/request_relative_speed_change' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_relative_speed_change
* Merge branch 'master' into matsuura/feature/add-time-to-panel
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#659 <https://github.com/tier4/scenario_simulator_v2/issues/659>`_ from tier4/release-0.6.1
* merge fix/galactic_build
* Merge branch 'fix/galactic_build' of https://github.com/tier4/scenario_simulator.auto into feature/request_relative_speed_change
* pull master
* merge master
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge pull request `#655 <https://github.com/tier4/scenario_simulator_v2/issues/655>`_ from tier4/fix/galactic_build
* use RCLCPP_INFO_STREAM macro
* coment out RCLCPP_INFO macro with fmt function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' into feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Contributors: MasayaKataoka, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge commit 'ce08abe39ed83d7ec0630560d293187fbcf08b5e' into feature/add_ideal_accel_model
* Merge pull request `#619 <https://github.com/tier4/scenario_simulator_v2/issues/619>`_ from RobotecAI/AJD-254-simple_abstract_scenario_for_simple_random_testing
* review changes application
* random test runner
* Contributors: Masaya Kataoka, Piotr Zyskowski, dai0912th
