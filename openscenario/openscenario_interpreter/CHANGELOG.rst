^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_interpreter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#295 <https://github.com/tier4/scenario_simulator_v2/issues/295>`_ from tier4/fix/python_format
  reformat by black
* reformat by black
* Merge pull request `#294 <https://github.com/tier4/scenario_simulator_v2/issues/294>`_ from tier4/feature/support-autoware.iv-0.11.2
  Feature/support autoware.iv 0.11.2
* Merge pull request `#292 <https://github.com/tier4/scenario_simulator_v2/issues/292>`_ from tier4/feature/ros_tooling_workflow
  use ros-setup action
* remove flake8 check
* Update to call setLaneChangeApproval only once
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/ros_tooling_workflow
* Merge pull request `#270 <https://github.com/tier4/scenario_simulator_v2/issues/270>`_ from tier4/feature/support-autoware.iv-0.11.1
  Feature/support autoware.iv 0.11.1
* Lipsticks
* Merge pull request `#287 <https://github.com/tier4/scenario_simulator_v2/issues/287>`_ from tier4/feature/remove-dummy-perception-publisher
  Feature/remove dummy perception publisher
* Rename package 'awapi_accessor' to 'concealer'
* Merge pull request `#281 <https://github.com/tier4/scenario_simulator_v2/issues/281>`_ from tier4/feature/asynchronous-autoware-initialization
  Feature/asynchronous autoware initialization
* Update Storyboard to call engage if Autoware is ready (= WaitingForEngage)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/update_contact_information
* Merge pull request `#276 <https://github.com/tier4/scenario_simulator_v2/issues/276>`_ from tier4/feature/autoware-high-level-api
  Feature/autoware high level api
* Move simulation specific topics into class MiscellaneousAPI
* Merge branch 'feature/support-autoware.iv-0.11.1' into feature/autoware-high-level-api
* Merge pull request `#274 <https://github.com/tier4/scenario_simulator_v2/issues/274>`_ from tier4/refactor/cleanup-ego-entity
  Refactor/cleanup ego entity
* Update entity_base::setDriverModel to be virtual
* Rename autoware_api::Accessor to awapi::Autoware
* Merge github.com:tier4/scenario_simulator.auto into feature/change_base_image
* Merge branch 'master' into feature/support-autoware.iv-0.11.1
* Merge pull request `#266 <https://github.com/tier4/scenario_simulator_v2/issues/266>`_ from tier4/feature/interpreter/traffic-signal-controller-3
  Feature/interpreter/traffic signal controller 3
* Lipsticks
* Add TrafficSignalControllerAction (dummy)
* Unlock InfrastructureAction
* Update TrafficSignalController to evaluate first phase
* Update readElement to return std::list instead of std::vector
* Lipsticks
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge branch 'master' into feature/interpolate_two_center_points
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-3
* Merge pull request `#263 <https://github.com/tier4/scenario_simulator_v2/issues/263>`_ from tier4/feature/traffic-signal-sensor
  Feature/traffic signal sensor
* Merge pull request `#264 <https://github.com/tier4/scenario_simulator_v2/issues/264>`_ from tier4/revert/interpolate_two_points
  Revert "enable interpolate two points"
* Revert "enable interpolate two points"
  This reverts commit 7b08f1d0de38e9b31e1d066d5c6ed7faec6758bd.
* enable interpolate two points
* Lipsticks
* Lipsticks
* Merge https://github.com/tier4/scenario_simulator.auto into doc/simple_sensor_simulator
* Merge pull request `#262 <https://github.com/tier4/scenario_simulator_v2/issues/262>`_ from tier4/feature/interpreter/traffic-signal-controller-2
  Feature/interpreter/traffic signal controller 2
* Lipsticks
* Update ScenarioDefinition to evaluate RoadNetwork for each frame
* Update TrafficSignalController (experimental)
* Update TrafficSignalController to use 'CircularIterator'
* Update TrafficSignalState to invoke API 'setTrafficLightColor'
* Fix Arrow and Color type's stream input operator
* Rename 'none' to 'blank' (T4v1 compatibility)
* Add new enumerated type 'Arrow' and 'Color'
* Update RoadNetwork::evaluate to invoke TrafficSignals's it
* Rename function 'setController' to 'assignController'
* Add local macro 'RENAME'
* Lipsticks
* Merge pull request `#260 <https://github.com/tier4/scenario_simulator_v2/issues/260>`_ from tier4/feature/interpreter/traffic-signal-controller
  Feature/interpreter/traffic signal controller 1
* Replace some of callWithElements with readElements
* Revert some changes
* Add free template function 'readElements'
* Cleanup function 'callWithElements'
* Rename helper template 'IfNotDefaultConstructible' to 'MustBe...'
* Merge pull request `#258 <https://github.com/tier4/scenario_simulator_v2/issues/258>`_ from tier4/fix/misc-problems
  Fix/misc problems
* Move struct 'TrafficSignalState' into header 'traffic_signal_state.hpp'
* Move some structs into header 'phase.hpp'
* Merge remote-tracking branch 'origin/fix/misc-problems' into feature/interpreter/traffic-signal-controller
* Move missing header includes
* Move some structs into header 'traffic_signal_controller.hpp'
* Lipsticks
* Lipsticks
* Add new type 'SpecialAction<N>' for CustomCommandAction
* Merge branch 'master' into fix/misc-problems
* Merge pull request `#238 <https://github.com/tier4/scenario_simulator_v2/issues/238>`_ from tier4/feature/interpreter/vehicle/base_link-offset
  Remove member function `API::spawn` receives XML strings.
* Cleanup catch clause
* Update openscenario_interpreter to dispatch AutowareError
* Merge remote-tracking branch 'origin/master' into feature/interpreter/vehicle/base_link-offset
* Update Pedestrian type to support cast operator
* Merge pull request `#257 <https://github.com/tier4/scenario_simulator_v2/issues/257>`_ from tier4/feature/rename_packages
  Feature/rename packages
* Update type Performance, Axles and Axle to support cast operator
* update namespace
* use clang_format
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* apply reformat
* Merge https://github.com/tier4/scenario_simulator.auto into feature/rename_packages
* Merge branch 'master' into doc/instructions
* Merge pull request `#255 <https://github.com/tier4/scenario_simulator_v2/issues/255>`_ from tier4/feature/interpreter/misc
  Feature/interpreter/misc
* rename simulation_api package
* Update WalkStraightAction to reveive actors as argument
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/text_lint
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* Merge pull request `#248 <https://github.com/tier4/scenario_simulator_v2/issues/248>`_ from tier4/feature/interpreter/pedestrian
  Feature/interpreter/pedestrian
* Add CustomCommandAction type 'WalkStraightAction'
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/walk_cartesian
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* Merge branch 'master' into feature/traffic_sink
* Merge pull request `#244 <https://github.com/tier4/scenario_simulator_v2/issues/244>`_ from tier4/feature/interpreter/assign-route-action
  Feature/interpreter/assign route action
* Update AssignRouteAction to work as StoryboardElement
* Update (Relative)?WorldPosition to support dummy cast operator
* Update Route type to support cast operator for requestAssignRoute
* Update Waypoint type to support cast operator for geometry_msgs::msg::Pose
* Merge branch 'master' into feature/interpreter/vehicle/base_link-offset
* Merge branch 'master' into feature/assign_route_action
* Merge pull request `#242 <https://github.com/tier4/scenario_simulator_v2/issues/242>`_ from tier4/feature/interpreter/remove-short-circuit-evaluation
  Feature/interpreter/remove short circuit evaluation
* Lipsticks
* Fix Trigger/ConditionGroup to evaluate rhs even if lhs is false
* Update Trigger/ConditionGroup to re-evaluate children for each evaluation
* Merge pull request `#240 <https://github.com/tier4/scenario_simulator_v2/issues/240>`_ from tier4/feature/interpreter/relative-distance-condition
  Feature/interpreter/relative distance condition
* Update some interfaces to use macro
* Update RelativeDistanceCondition to use function 'getBoundingBoxDistance'
* Add cast operator for geometry_msgs::msg::Vector3 to type Dimensions
* Move BoundingBox's stream output operator into .cpp file
* Add cast operator for geometry_msgs::msg::Point to Center type
* Move VehicleCategory's stream I/O operators into .cpp file
* Add helper function 'apply' to dispatch EntityObject dynamically
* Lipsticks
* Merge branch 'master' into fix/reindex-rtree
* Merge pull request `#234 <https://github.com/tier4/scenario_simulator_v2/issues/234>`_ from tier4/feature/interpreter/teleport-action
  Feature/interpreter/teleport action
* Update LanePosition's cast operator to be marked explicit
* Update ReachPositionCondition to use helper function 'apply'
* Update TeleportAction to support WorldPosition
* Update helper function 'apply' to dispatch position types dinamically
* Replace constructLaneletPose with LanePosition type's cast operator
* Lipsticks
* Merge pull request `#233 <https://github.com/tier4/scenario_simulator_v2/issues/233>`_ from tier4/feature/interpreter/misc
  Feature/interpreter/misc
* Remove accidentially commited file
* Add member function 'EgoEntity::initializeAutoware'
* Add member function 'Accessor::setInitialVelocity'
* Simplify member function API::setTargetSpeed
* Cleanup SpeedAction
* Update RelativeDistanceCondition to print debug informations
* Update SpeedCondition to print debug informations
* Cleanup procedures
* Remove deprecated member function 'toPose'
* Update LanePosition type to support cast operator for geometry_msgs::msg::Pose
* Remove LanePosition's cast operator for type 'geometry_msgs::msg::Pose'
* Update ReachPositionCondition to print some debug information
* Update SimulationTimeCondition's debug printer
* Lipsticks
* Lipsticks
* Merge pull request `#232 <https://github.com/tier4/scenario_simulator_v2/issues/232>`_ from tier4/misc
  Misc
* Fix ScenarioObject to attach sensors if is Ego
* Merge pull request `#229 <https://github.com/tier4/scenario_simulator_v2/issues/229>`_ from tier4/feature/test-runner/autoware.launch.xml
  Feature/test runner/autoware.launch.xml
* Cleanup openscenario_interpreter.cpp
* Rename some parameters
* Merge branch 'master' into feature/test-runner/autoware.launch.xml
* Merge pull request `#231 <https://github.com/tier4/scenario_simulator_v2/issues/231>`_ from tier4/feature/add_contributing_md
  Feature/fix_licence_problems
* modify package.xml
* Merge branch 'master' into doc/zeromq
* Merge pull request `#227 <https://github.com/tier4/scenario_simulator_v2/issues/227>`_ from tier4/feature/interpreter/object-controller
  Feature/interpreter/object controller
* Update Controller.Properties to support property 'isEgo'
* Merge pull request `#225 <https://github.com/tier4/scenario_simulator_v2/issues/225>`_ from tier4/feature/support-autoware.iv-9
  Feature/support autoware.iv 9
* Fix TrafficSignalController.delay to be optional
* Fix raycasting
* Update API class to receive scenario path as argument
* Merge https://github.com/tier4/scenario_simulator.auto into doc/zeromq
* Merge pull request `#222 <https://github.com/tier4/scenario_simulator_v2/issues/222>`_ from tier4/feature/interpreter/sticky
  Feature/interpreter/sticky
* Add condition edge 'sticky'
* Lipsticks
* Merge pull request `#214 <https://github.com/tier4/scenario_simulator_v2/issues/214>`_ from tier4/feature/support-autoware.iv-5
  Feature/support autoware.iv 5
* Lipsticks
* Rename member function 'guard' to 'withExceptionHandler'
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into fix/xmlrpc_connection_lost
* Merge pull request `#206 <https://github.com/tier4/scenario_simulator_v2/issues/206>`_ from tier4/fix/node-duplication
  Fix/node duplication
* Update nodes namespace
* Merge branch 'master' into feature/get_waypoint_from_autoware
* Merge branch 'master' into feature/get_waypoint_from_autoware
* Merge pull request `#197 <https://github.com/tier4/scenario_simulator_v2/issues/197>`_ from tier4/feature/support-autoware.iv-2
  Feature/support autoware.iv 2
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Merge pull request `#200 <https://github.com/tier4/scenario_simulator_v2/issues/200>`_ from tier4/feature/lidar_simulation
  Feature/lidar simulation
* fix typo
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
* Update to entity spawn to refer VehicleProperty
* Update ObjectController to be DefaultConstructible
* Add class 'Controller'
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv-2
* Update ego entity's member function 'onUpdate'
* Merge pull request `#196 <https://github.com/tier4/scenario_simulator_v2/issues/196>`_ from tier4/feature/despawn_entity
  Feature/despawn entity
* enable despawn entity in sim
* Update awapi_accessor to be Node
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/use_protobuf_in_spawn
* Merge pull request `#179 <https://github.com/tier4/scenario_simulator_v2/issues/179>`_ from tier4/feature/support-autoware.iv
  Feature/support autoware.iv
* Convert Accessor to member variable from base class
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/protobuf_xmlrpc
* Merge branch 'master' into feature/support-autoware.iv
* Merge pull request `#191 <https://github.com/tier4/scenario_simulator_v2/issues/191>`_ from tier4/feature/interpreter/property
  Feature/interpreter/property
* Remove property 'isEgo' from some test scenarios
* Add property 'isEgo'
* Update class ScenarioObject
* Lipsticks
* Add syntax 'Property'
* Add syntax 'Property'
* Lipsticks
* Merge remote-tracking branch 'origin/master' into feature/support-autoware.iv
* Merge pull request `#174 <https://github.com/tier4/scenario_simulator_v2/issues/174>`_ from tier4/feature/destroy-entity-action
  Feature/destroy entity action
* Merge branch 'fix/metrics' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_crosswalk
* Remove debug print
* Update test scenario 'simple.xosc'
* Support 'AddEntityAction'
* Support 'DeleteEntityAction'
* Support 'EntityAction'
* Update Action type's constructor
* Merge pull request `#167 <https://github.com/tier4/scenario_simulator_v2/issues/167>`_ from tier4/feature/custom-command-action
  Feature/custom command action
* Lipsticks
* Cleanup
* Update CustomCommandAction to accept C-style function syntax
* Add experimental custom-command-parser
* Lipsticks
* Update function 'fork_exec'
* Move function 'split' into header 'string/split.hpp'
* Add free function 'fork_exec'
* Move function 'execvp' into header 'posix/fork_exec.hpp'
* Merge pull request `#149 <https://github.com/tier4/scenario_simulator_v2/issues/149>`_ from tier4/feature/foxy
  Feature/foxy
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Fix dependency.repos
* Merge pull request `#150 <https://github.com/tier4/scenario_simulator_v2/issues/150>`_ from tier4/feature/entity_waypoint
  Feature/entity waypoint
* remove failure scenario
* Update option 'global-timeout' to disable if None specified
* Lipsticks
* Add optional argument verbose (= True) to openscenario_utility
* Rename 'step_time_ms' to 'frame-rate'
* Fix external/quaternion_operation's commit hash
* Add parameter 'real-time-factor' to openscenario_interpreter
* enable pass colcon test
* enable pass test case
* Revert some changes
* Reformat openscenario/*
* change default parameters
* Update 'CMakeLists.txt's to use 'ament_auto*' macros
* Merge remote-tracking branch 'origin/master' into feature/foxy
* Merge branch 'master' into refactor/scenario-test-runner-2
* Merge pull request `#147 <https://github.com/tier4/scenario_simulator_v2/issues/147>`_ from tier4/feature/remove_entity_status
  Feature/remove entity status
* apply reformat
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_entity_status
* retry tick when the action is root
* Merge pull request `#146 <https://github.com/tier4/scenario_simulator_v2/issues/146>`_ from tier4/refactor/scenario-test-runner
  Refactor/scenario test runner
* enable compile interpritor
* add openscenario_msgs to the depends
* Lipsticks
* Merge branch 'master' into feature/collision_to_hermite_curve
* Merge pull request `#128 <https://github.com/tier4/scenario_simulator_v2/issues/128>`_ from tier4/feature/ordered-xosc
  Feature/ordered xosc
* Merge branch 'master' into feature/ordered-xosc
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/remove_spline_interpolation
* Merge pull request `#133 <https://github.com/tier4/scenario_simulator_v2/issues/133>`_ from tier4/feature/relative-world-position
  Feature/relative world position
* Fix convertion to rpy from hpr
* Remove accidentially committed files
* Lipsticks
* Support Teleport to RelativeWorldPosition
* Update Orientation type to support implict cast to Vector3
* Add utility function 'fold\_(left|right)' and 'cat'
* Support new position category 'RelativeWorldPosition'
* Merge branch 'master' into feature/spawn_relative_entity_position
* Convert some staticmethods to be free function
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/catmull-rom
* Merge pull request `#118 <https://github.com/tier4/scenario_simulator_v2/issues/118>`_ from tier4/document/openscenario_interpreter
  Document/openscenario interpreter
* Update documentation
* Update substitution syntax description
* Merge branch 'master' into doc/test_runner
* Merge pull request `#117 <https://github.com/tier4/scenario_simulator_v2/issues/117>`_ from tier4/document/openscenario_interpreter
  Document/openscenario interpreter
* Fix company name
* Merge branch 'master' into feature/awapi_adapter/add_info
* Merge branch 'master' into feature/awapi_adapter/vehicle_info
* Merge pull request `#94 <https://github.com/tier4/scenario_simulator_v2/issues/94>`_ from tier4/fix/contact_infomation
  Fix/contact infomation
* Merge remote-tracking branch 'origin/master' into feature/awapi_adapter/add_info
* Merge branch 'master' into feature/awapi_adapter/vehicle_info
* Merge pull request `#93 <https://github.com/tier4/scenario_simulator_v2/issues/93>`_ from tier4/fix/copyright
  update copyright
* fix contact infomation of tatsuya yamasaki
* update copyright
* Merge remote-tracking branch 'origin/master' into feature/awapi_adapter/lib
* Merge branch 'master' into feature/awapi_awauto_adapter
* Merge pull request `#86 <https://github.com/tier4/scenario_simulator_v2/issues/86>`_ from tier4/feature/ego_vehicle
  Feature/ego vehicle
* update API
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into documentation/simulation_api
* Merge branch 'master' into feature/awapi_awauto_adapter
* Merge branch 'master' into documentation/simulation_api
* Merge pull request `#82 <https://github.com/tier4/scenario_simulator_v2/issues/82>`_ from tier4/feature/awapi_accessor
  Feature/awapi accessor
* Cleanup CMakeLists.txt
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into documentation/simulation_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into fix/collision
* Merge pull request `#74 <https://github.com/tier4/scenario_simulator_v2/issues/74>`_ from tier4/feature/procedures
  Feature/procedures
* Fix error-message
* Add new test scenario 'distance-condition.yaml'
* Merge branch 'feature/procedures' of https://github.com/tier4/scenario_simulator.auto into fix/collision
* Fix error-message
* Support CollisionCondition (UNTESTED)
* Support DistanceCondition for WorldPosition
* Add class 'DistanceCondition'
* Lipsticks
* Move class template 'equal_to' into header 'equal_to.hpp'
* Support 'StandStillCondition'
* Merge pull request `#73 <https://github.com/tier4/scenario_simulator_v2/issues/73>`_ from tier4/feature/back_run
  Feature/back run
* configure parameters
* add calculateEntityStatusUpdatedInWorldFrame function
* enable pass scenario test
* Merge branch 'master' into fix/documentation
* Merge pull request `#71 <https://github.com/tier4/scenario_simulator_v2/issues/71>`_ from tier4/feature/parameter
  Feature/parameter
* Rename word from 'open_scenario' to 'openscenario'
* Contributors: Makoto Tokunaga, Masaya Kataoka, Tatsuya Yamasaki, Yamasaki Tatsuya, taikitanaka, taikitanaka3, yamacir-kit
