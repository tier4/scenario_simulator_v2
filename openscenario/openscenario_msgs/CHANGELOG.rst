^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2021-07-13)
------------------
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge branch 'master' into traffic_signal_actions
* Merge pull request `#380 <https://github.com/tier4/scenario_simulator_v2/issues/380>`_ from tier4/feature/misc_object
* add MiscObjectParameters message type
* add Misc Object type
* Merge branch 'master' into traffic_signal_actions
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718

0.2.0 (2021-06-24)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka

0.1.1 (2021-06-21)
------------------
* Merge pull request `#344 <https://github.com/tier4/scenario_simulator_v2/issues/344>`_ from tier4/feature/interpreter/context
* Add new package 'openscenario_interpreter_msgs'
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Contributors: Masaya Kataoka, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge remote-tracking branch 'origin/master' into fix/traffic-simulator/simulation-model-2
* Merge pull request `#349 <https://github.com/tier4/scenario_simulator_v2/issues/349>`_ from tier4/fix/typos-in-openscenario-dir
* Fix typos in the openscenario directory
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Contributors: Kazuki Miyahara, Masaya Kataoka, yamacir-kit

0.0.1 (2021-05-12)
------------------
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Merge branch 'master' into fix/misc-problems
* Merge pull request `#238 <https://github.com/tier4/scenario_simulator_v2/issues/238>`_ from tier4/feature/interpreter/vehicle/base_link-offset
  Remove member function `API::spawn` receives XML strings.
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* Update type Performance, Axles and Axle to support cast operator
* use clang_format
* Add cast operator for geometry_msgs::msg::Point to Center type
* Move VehicleCategory's stream I/O operators into .cpp file
* move some headers
* move some logics
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#195 <https://github.com/tier4/scenario_simulator_v2/issues/195>`_ from tier4/feature/use_protobuf_in_spawn
  Feature/use protobuf in spawn
* remove is_ego parameter
* fix typo
* add parameter message
* add vehicle parameters message
* add Property message
* add axles message
* add message
* add performance message
* Merge branch 'master' into feature/support-autoware.iv
* Merge branch 'master' into feature/awapi-accessor/non-awapi-topics
* Merge pull request `#188 <https://github.com/tier4/scenario_simulator_v2/issues/188>`_ from tier4/feature/idiot_npc
  Feature/idiot npc
* update driver model
* Merge pull request `#162 <https://github.com/tier4/scenario_simulator_v2/issues/162>`_ from tier4/feature/driver_model
  Feature/driver model
* add comment
* add driver model message
* Merge pull request `#159 <https://github.com/tier4/scenario_simulator_v2/issues/159>`_ from tier4/feature/obstacle_visualization
  Feature/obstacle visualization
* enable get obstacle via entity manager
* modify obstacle message
* modify obstacle message
* add obstacle output port
* add obstacle type
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge pull request `#150 <https://github.com/tier4/scenario_simulator_v2/issues/150>`_ from tier4/feature/entity_waypoint
  Feature/entity waypoint
* add calculate obstacles function
* configure message
* enable calculate waypoint while lane follow
* add waypoint message type
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge branch 'master' into refactor/scenario-test-runner-2
* add trajectory message type
* add curve message
* Merge pull request `#147 <https://github.com/tier4/scenario_simulator_v2/issues/147>`_ from tier4/feature/remove_entity_status
  Feature/remove entity status
* Merge branch 'feature/remove_entity_status' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* reduce compile errors
* add EntityType message
* Merge branch 'master' into feature/ordered-xosc
* Merge pull request `#140 <https://github.com/tier4/scenario_simulator_v2/issues/140>`_ from tier4/feature/lanlet_pose
  Feature/lanlet pose
* modify message
* remove twist/accel from entity status
* add message types
* add LaneletPose message
* Merge branch 'master' into feature/test_runner/add_implementation_details
* Merge branch 'master' into feature/remove-github-access-token
* Merge branch 'master' into refactor/converter
* Merge pull request `#65 <https://github.com/tier4/scenario_simulator_v2/issues/65>`_ from tier4/feature/yield
  Feature/yield
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/error-handling
* change lanlet id type from int to int64
* Merge pull request `#59 <https://github.com/tier4/scenario_simulator_v2/issues/59>`_ from tier4/feature/enhance_visualization
  Feature/enhance visualization
* add text action marker
* enable publish bounding box
* add openscenario visualization directory
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, yamacir-kit
