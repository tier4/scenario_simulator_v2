^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simulation_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2021-07-13)
------------------
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* enable send SpawnMiscObjectEntityRequest to the sensor simulator
* update zeromq server
* fix test code in tntity type
* add test cases
* add toProto toMsg functions
* add MiscObjectParameters message type
* add Misc Object type
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718

0.2.0 (2021-06-24)
------------------
* Merge pull request `#357 <https://github.com/tier4/scenario_simulator_v2/issues/357>`_ from tier4/feature/send_ego_command
* fix typo
* fix indent
* fix indent
* fix indend
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* apply clang-format
* add proto files
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
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
* Merge pull request `#306 <https://github.com/tier4/scenario_simulator_v2/issues/306>`_ from tier4/feature/use_common_exception
* Remove trailing semicolon from macro definition
* remove protobuf conversion error
* remove unused macro
* remove XML parameter error
* remove SimulationClockError
* remove Execution Error
* Merge pull request `#304 <https://github.com/tier4/scenario_simulator_v2/issues/304>`_ from tier4/feature/synchronize_clock
* modify API schema
* enable convert clock topic
* add Clock message definition
* add messages
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
* remove flake8 check
* add new line for the block
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* use clang_format
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* apply reformat
* Merge pull request `#252 <https://github.com/tier4/scenario_simulator_v2/issues/252>`_ from tier4/feature/publish_proto_doc
  Feature/publish proto doc
* enable pass textlint
* udpate proto
* modify case
* enable generate link
* add document about openscenario_msgs.proto
* add document comment
* update docs
* fix case
* remove unusd proto files
* Merge branch 'master' into feature/test-runner/autoware.launch.xml
* Merge pull request `#231 <https://github.com/tier4/scenario_simulator_v2/issues/231>`_ from tier4/feature/add_contributing_md
  Feature/fix_licence_problems
* fix e-mail
* modify package.xml
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-8
* Merge pull request `#219 <https://github.com/tier4/scenario_simulator_v2/issues/219>`_ from tier4/feature/remove_xmlrpc
  Feature/remove xmlrpc
* Merge branch 'master' into feature/remove_xmlrpc
* Merge pull request `#217 <https://github.com/tier4/scenario_simulator_v2/issues/217>`_ from tier4/feature/zeromq_integration
  Feature/zeromq integration
* add default
* enable pass linter
* remove stop watch
* add all methods
* add lidar callback
* add despawn entity gunction
* add spawn pedestrian func
* add spawn vehicle entity
* add soe callbacks
* add multi server
* remove debuf lines
* add stop watch
* add stopwatch class
* update constants
* remove unused files
* check transport repeated field
* remove xmlrclcpp
* enable pass cpplint
* apply reformat
* add port configuration
* add constructor to the server
* add hostname enum
* add constructor
* Merge branch 'master' into feature/support-autoware.iv-6
* Merge pull request `#216 <https://github.com/tier4/scenario_simulator_v2/issues/216>`_ from tier4/fix/test_in_simulation_interface
  Fix/test in simulation interface
* enable pass flake8
* apply reformat
* Merge branch 'master' into feature/support-autoware.iv-6
* enable call client
* add debug line
* add debug line
* add launch file
* Merge branch 'master' into feature/support-autoware.iv-4
* Merge pull request `#207 <https://github.com/tier4/scenario_simulator_v2/issues/207>`_ from tier4/fix/xmlrpc_connection_lost
  Fix/xmlrpc connection lost
* enable set status
* remove unused fields
* fix scenario
* add debug lines
* remove unused lines
* remove multicall
* remove warnings
* remove multicall
* Merge pull request `#202 <https://github.com/tier4/scenario_simulator_v2/issues/202>`_ from tier4/feature/get_waypoint_from_autoware
  Feature/get waypoint from autoware
* add proto
* Merge branch 'feature/publish_obstacle' of https://github.com/tier4/scenario_simulator.auto into feature/get_waypoint_from_autoware
* modify include gurad
* rename packages
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, yamacir-kit
