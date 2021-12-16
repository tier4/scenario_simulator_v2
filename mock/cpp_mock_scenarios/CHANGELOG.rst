^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cpp_mock_scenarios
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* move to mock pacakge
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
