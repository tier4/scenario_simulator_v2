^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_sensor_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
