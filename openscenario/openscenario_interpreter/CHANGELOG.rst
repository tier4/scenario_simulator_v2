^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_interpreter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2023-07-26)
------------------
* Merge pull request `#1028 <https://github.com/tier4/scenario_simulator_v2/issues/1028>`_ from tier4/pzyskowski/660/zmq-interface-change-impl
* Merge pull request `#1037 <https://github.com/tier4/scenario_simulator_v2/issues/1037>`_ from tier4/fix/junit-missing-count
* Fix: JUnit format output contain wrong case count
* do nothing plugin fix
* clang format
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change-impl
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#906 <https://github.com/tier4/scenario_simulator_v2/issues/906>`_ from tier4/feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge pull request `#1013 <https://github.com/tier4/scenario_simulator_v2/issues/1013>`_ from tier4/feature/rtc_custom_command_action
* Update accomplished to return false if it is the first call after start
* Update `follow_trajectory::Parameter` to hold base time
* Rename data member `Parameter<>::timing_is_absolute`
* chore: revert CMakeLists.txt in openscenario_interpreter
* refactor: define sendCooperateCommand in NonStandardOperation
* Update `FollowTrajectoryAction::accomplished` to work correctly
* feat: implement RequestToCorporateCommandAction
* feat(concealer): implement rtc module name conversion
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/zmq-interface-change
* Lipsticks
* feat(openscenario_interpreter): add empty implementation of ApplyRequestToCorporateAction
* Update to properly calculate remaining time when timing is relative
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge remote-tracking branch 'tier4/master' into pzyskowski/660/ego-entity-split
* Merge pull request `#1004 <https://github.com/tier4/scenario_simulator_v2/issues/1004>`_ from tier4/feat/v2i_custom_command_action
* pedestrian and misc object models passed
* model3d sent via zmq
* refactor(traffic_simulator, openscenario_interpreter): unify usage timing of plural forms
* refactor(traffic_simulator, openscenario_interpreter): use reset instead of apply
* refactor(traffic_simulator, openscenario_interpreter): rename some variable & function name
* chore(openscenario_interpreter): fix order of command list of CustomCommandAction
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* refactor(openscenario_interpreter): ApplyV2ISignalStateAction::start
* Merge pull request `#1002 <https://github.com/tier4/scenario_simulator_v2/issues/1002>`_ from tier4/feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* refactor: rename traffic_relation to traffic_lights
* refactor: move method related to traffic signals from CoordinateConversion to NonStandardOperation
* fix: typo
* Merge branch 'pzyskowski/660/ego-entity-split' into pzyskowski/660/zmq-interface-change
* feat: implement update publish rate for V2ITrafficSignalState
* Update `FunctionCallExpression` to accept `@` as part of function name
* Cleanup `ApplyFaultInjectionAction<>::start`
* Merge remote-tracking branch 'origin/master' into feat/v2i_custom_command_action
* feat(traffic_simulator): implement update publish rate function for traffic lights
* refactor(traffic_simulator): forward getTrafficLights function to each type of traffic lights
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Lipsticks
* Add experimental custom command `FaultInjectionAction@v2`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Lipsticks
* Merge remote-tracking branch 'origin/master' into fix/get_s_value
* Merge pull request `#969 <https://github.com/tier4/scenario_simulator_v2/issues/969>`_ from RobotecAI/pzyskowski/660/concealer-split
* feat(traffic_simulator): enable to set state in V2ITrafficSignalStateAction
* Delete prints that are worthless to the user
* feat(traffic_simulator): add empty implementation of V2ITrafficSignalStateAction
* Update `behavior_plugin` to receive Parameter via `shared_ptr`
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* clang format
* applied AutowareUser name change to FOA
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge branch 'pzyskowski/660/concealer-split' into pzyskowski/660/ego-entity-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'tier/master' into pzyskowski/660/concealer-split
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Update `FollowPolylineTrajectoryAction` to work if `time` unspecified
* Update `FollowPolylineTrajectoryAction` to receive parameter
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/follow-trajectory-action
* Lipsticks
* Update syntax `FollowTrajectoryAction` to request behavior `FollowTrajectory`
* Rename data member `Polyline::vertex` to `vertices`
* Update syntax `FollowTrajectoryAction` to hold `Scope`
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, Piotr Zyskowski, Tatsuya Yamasaki, f0reachARR, yamacir-kit

0.6.8 (2023-05-09)
------------------
* Merge pull request `#990 <https://github.com/tier4/scenario_simulator_v2/issues/990>`_ from tier4/fix/cspell_errors
* apply linter
* docs: use ROS 2 instead of ROS2
* Merge pull request `#979 <https://github.com/tier4/scenario_simulator_v2/issues/979>`_ from RobotecAI/ref/AJD-696_clean_up_metics_traffic_sim
* ref(simulator_core): apply ament_clang_reformat
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge pull request `#980 <https://github.com/tier4/scenario_simulator_v2/issues/980>`_ from tier4/feature/interpreter/environment
* Merge pull request `#894 <https://github.com/tier4/scenario_simulator_v2/issues/894>`_ from tier4/fix/cleanup_code
* Merge branch 'master' into feature/interpreter/environment
* Merge pull request `#986 <https://github.com/tier4/scenario_simulator_v2/issues/986>`_ from tier4/feature/interpreter/publishing-delay
* Merge pull request `#977 <https://github.com/tier4/scenario_simulator_v2/issues/977>`_ from tier4/feature/interpreter/model3d-field
* Merge branch 'master' into feature/interpreter/model3d-field
* Update AssignControllerAction to read property `detectedObjectPublishingDelay`
* Lipsticks
* Revert "feat(traffic_sim): add setJerkLimit"
* Fix optional types default
* Fix code styles
* Revert unexpected formatting change
* Use default constructor instead of optional with warnings
* Revert "Add optional support for  readElement"
* Revert "Add optional support for readAttribute"
* Use if initialization on validate
* Remove done todo
* Fix code styles
* Show tag name on attribute error message
* Fix code styles
* Use default constructor on Percipitation
* Merge remote-tracking branch 'origin/master' into feature/interpreter/model3d-field
* Fix misuse constant
* Merge branch 'master' into feature/interpreter/environment
* Fix code style
* Add range checker on Environment related element
* Use optional on Weather
* Add optional support for readAttribute
* Use optional by readElement on Environment
* Add optional support for  readElement
* Use boost::optional for indicate element is empty
* Add missing member to Environment
* Add RoadCondition tag syntax
* Add Wetness attribute syntax
* Fix code styles
* feat(traffic_sim): add out_of_range as job actvated in AddEntityAction functor
* Merge branch 'master' into ref/AJD-696_clean_up_metics_traffic_sim
* Merge branch 'master' into fix/cleanup_code
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into ref/AJD-697_improve_port_management_zmq
* Merge pull request `#971 <https://github.com/tier4/scenario_simulator_v2/issues/971>`_ from tier4/feature/interpreter/delay_in_condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/delay_in_condition
* feat(traffic_sim): ensure max_jerk as variable in entity_base
* Revert "feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit"
* Merge pull request `#978 <https://github.com/tier4/scenario_simulator_v2/issues/978>`_ from tier4/feature/interpreter/relative-heading-condition
* Merge remote-tracking branch 'origin/master' into clean-dicts
* Fix code style
* Add DomeImage tag syntax
* Add copyright notice to time_of_day.cpp
* Add Wind tag syntax
* Add Precipitation tag to Weather
* Add Precipitation tag syntax
* Add PrecipitationType attribute syntax
* Fix XSD comment
* Add first Weather tag syntax
* Merge branch 'master' into feature/interpreter/model3d-field
* Add OpenSCENARIO version
* Add range information for Fog
* Fix class member naming
* Add Fog tag syntax
* Add Sun tag syntax
* Add FractionalCloudCover attribute type
* Add TimeOfDay tag syntax
* Add ParameterDeclarations child to Environment
* Add Environment to Catalog
* Add EnvironmentAction syntax to GlobalAction
* Add EnvironmentAction syntax
* Add Environment base syntax
* Merge remote-tracking branch 'origin/master' into ref/AJD-696_clean_up_metics_traffic_sim
* Use `typename` instead of `class`
* Format
* Use static_cast instead of dynamic initialization
* Use more precise function signature
* Remove trailing return type from lambda function
* Update signature of lambda functions
* Merge branch 'master' into feature/noise_delay_object
* Change not to use `std::reverse_iterator`
* Fix wrong evaluation order
* Refactor `update_condition`
* Update `Condition::evaluate`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/relative-heading-condition
* Merge pull request `#975 <https://github.com/tier4/scenario_simulator_v2/issues/975>`_ from tier4/emergency-state/backwardcompatibility-1
* refactor(concealer)
* Merge branch 'master' into feature/noise_delay_object
* feat(traffic_sim): add max_jerk, maxJerk, setJerkLimit
* Update sample scenario to use one-argument version of `RelativeHeadingCondition`
* Update `RelativeHeadingCondition` to work even if only one argument is given
* Add `model3d` attribute to entity object
* ref(traffic_sim):  out_of_range  only for npc vehicles, add tolerance
* ref(traffic_simulator): remove out_of_range metric
* Merge remote-tracking branch 'origin/master' into emergency-state/backwardcompatibility-1
* Merge remote-tracking branch 'origin/emergency-state/latest' into emergency-state/backwardcompatibility-1
* Merge pull request `#973 <https://github.com/tier4/scenario_simulator_v2/issues/973>`_ from tier4/feature/interpreter/probability-of-lost
* Update to accept all condition edges
* Lipsticks
* Update `applyAssignControllerAction` to set `probability_of_lost` to detection sensor
* Merge remote-tracking branch 'origin' into feature/add_setgoalposes_api
* Use `test()` instead of subscript operator
* Fix wrong comparison
* Merge remote-tracking branch 'origin/master' into fix/openscenario_utility/conversion
* Merge pull request `#874 <https://github.com/tier4/scenario_simulator_v2/issues/874>`_ from tier4/feature/interpreter/user-defined-value-condition
* Update wording
* Fix typo
* Refactor `evaluate` implementation
* Merge remote-tracking branch 'origin/master' into fix/get-unique-route-lanelets
* Merge pull request `#957 <https://github.com/tier4/scenario_simulator_v2/issues/957>`_ from tier4/feature/interpreter/license_and_properties
* Fix wrong comparison
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge branch 'master' into fix/cleanup_code
* Format
* Add condition edge implementation
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Add `delay` functionality for `condition`
* Format
* Update `License` definition
* Merge remote-tracking branch 'origin/master' into fix/get-unique-route-lanelets
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#967 <https://github.com/tier4/scenario_simulator_v2/issues/967>`_ from RobotecAI/fix/AJD-655-terminates-sigint
* Merge pull request `#932 <https://github.com/tier4/scenario_simulator_v2/issues/932>`_ from tier4/feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Move a message type `UserDefinedValue` to an external package `tier4_simulation_msgs`
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge pull request `#966 <https://github.com/tier4/scenario_simulator_v2/issues/966>`_ from RobotecAI/fix/AJD-653-map-path-files
* Update ReleaseNotes
* reformat: apply ament_clang_format
* refactor: apply ament_clang_format
* use __has_include
* modify CMakeLists.txt
* fix(os_interp): fix abort caused by ~Interpreter
* fix(sim_core): fix getting the eval sim time
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin' into fix/getting_next_lanelet
* Merge remote-tracking branch 'origin/master' into feature/interpreter/license_and_properties
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#965 <https://github.com/tier4/scenario_simulator_v2/issues/965>`_ from tier4/emergency-state/latest-behavior
* Merge pull request `#962 <https://github.com/tier4/scenario_simulator_v2/issues/962>`_ from tier4/feature/interpreter/relative-distance-condition
* Merge pull request `#954 <https://github.com/tier4/scenario_simulator_v2/issues/954>`_ from tier4/fix/python-installation
* feat(interpreter): implement MRM behavior interface
* chore: delete commented line in package.xml
* Cleanup
* Update `DistanceCondition` to support lateral lane-coordinate distance
* Update `RelativeDistanceCondition` to support lateral lane-coordinate distance
* chore: add debug messages
* Lipsticks
* Format
* Make `resource` and `spdxId` optional
* Add License and Properties fields
* Remove unused parameter
* feat(interpreter): implement EmergencyState interface for interpreter
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Add new command `features`
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* chore: revert condition name
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into emergency-state/backward-compatibility
* feat(UserDefinedCondition): add `currentMrmState` and `currentMrmBehavior` when there is　autoware_adapi_v1_msgs
* chore: add autoware_adapi_v1_msgs as an option
* Update `MultiServer` to be monitored by `status_monitor`
* Remove `status_monitor` from `openscenario_interpreter_node`
* Update struct `Status` to hold first caller name
* Merge remote-tracking branch 'origin/master' into import/universe-2437
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#937 <https://github.com/tier4/scenario_simulator_v2/issues/937>`_ from tier4/feature/interpreter/noise
* Update `StatusMonitor` to check elapsed time since last access
* Update `AssignControllerAction` to consider some object detection properties
* Update `rand_engine\_` to allocate on the stack instead of the heap
* Lipsticks
* Merge branch 'master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/alive-monitoring
* Merge pull request `#926 <https://github.com/tier4/scenario_simulator_v2/issues/926>`_ from tier4/feature/empty/parameter_value_distribution-fixed
* Update `CMakeLists.txt` to not build `libopenscenario_interpreter.so` as component
* Add new struct `openscenario_interpreter::LifecycleNode`
* refactor(interpreter): use static_cast to clarify what the code is meant
* refactor(interpreter): initialize Scope::seed with loaded value from scenario
* refactor(interpreter): use Scope::seed
* Merge remote-tracking branch 'origin/master' into feature/traveled_distance_as_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge pull request `#928 <https://github.com/tier4/scenario_simulator_v2/issues/928>`_ from tier4/feature/interpreter/speed-profile-action
* refactor(interpreter): initialize randomSeed with 0
* Merge branch 'master' into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* chore(interpreter): apply clang-format
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* refactor(interpreter): avoid copy and use perfect forwarding
* refactor(interpreter): rename template name
* Remove deprecated static member function `ActionApplication::getBehaviorParameter`
* Cleanup member function `SpeedProfileAction::run`
* Remove debug printings from `SpeedProfileAction`
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge pull request `#909 <https://github.com/tier4/scenario_simulator_v2/issues/909>`_ from tier4/feature/jerk_planning
* Revert "Revert "refactor(interpreter): delete unnecessary this keyword""
* chore(interpreter): fix compile error
* refactor(interpreter): rename variable
* Revert "refactor(interpreter): delete unnecessary this keyword"
* refactor(interpreter): make a name of member variable that is used as a function, like a function
* refactor(interpreter): delete unnecessary this keyword
* refactor(interpreter): make name for instance of StochasticDistributionSampler like a function
* refactor(interpreter): use explicit keyword for struct constructor
* chore(interpreter): fix implementation of  ProbabilityDistributionSet::evaluate()
* feat(interpreter): implement ProbabilityDistributionSet::evaluate()
* refactor(interpreter): delete unused member in UserDefinedDistribution
* chore(interpreter): fix compile error
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* feat(interpreter): implement evaluate functions for stochastic distribution classes
* refactor(interpreter): rename StochasticDistributionClass into StochasticDistributionSampler
* Update `SpeedProfileAction` debug informations
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* refactor(interpreter): delete unused inheritance
* fix(interpreter): add missing attribute to DeterministicSingleParameterDistribution
* feat(interpreter): add ParameterValueDistributionDefinition
* Update `SpeedProfileAction` to select `Transition` by attribute `followingMode`
* Added debug prints to `SpeedProfileAction`
* Update SpeedProfileAction to call `requestSpeedChange` with `Transition::AUTO`
* Add new function `getBehaviorParameter` for debug
* Rename local function `compare` to `accomplished`
* fix!(UserDefinedValueCondition): convert mrm state to emergency state
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* chore(interpreter): update error messages
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution-fixed
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/simple_noise_simulator
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* refactor(interpreter): move distribution.hpp into `random` directory
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into feature/add_setgoalposes_api
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/jerk_planning
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* fix compile errors
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Fix wrong function call
* Merge remote-tracking branch 'origin/master' into fix/cleanup_code
* Format
* Replace boost::optional with std::optional
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Merge remote-tracking branch 'origin/master' into feature/improve_occupancy_grid_algorithm
* Update some nodes to treat `scenario_simulator_v2_msgs` to be required dependency
* Update `UserDefinedValueCondition` to be compilable even if there is no package named `scenario_simulator_v2_msgs`
* Update `UserDefinedValueCondition` to use message of package `scenario_simulator_v2_msgs`
* Update `UserDefinedValueCondition` to not to depend `ParameterDeclaration`
* Remove data member `name` from `openscenario_msgs::msg::ParameterDeclaration`
* Remove `openscenario_msgs::msg::ValueConstraint` and `ValueConstraintGroup`
* Rename data member `evaluateValue` to `evaluate_value`
* Merge branch 'master' into feature/improve_occupancy_grid_algorithm
* Lipsticks
* Merge branch 'master' into fix_wrong_merge
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/use_job_in_standstill_duration
* Contributors: Dawid Moszyński, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Michał Kiełczykowski, Minami Shota, Shota Minami, Tatsuya Yamasaki, f0reachARR, hrjp, kyoichi-sugahara, yamacir-kit, ぐるぐる

0.6.7 (2022-11-17)
------------------
* chore(interpreter): fix commit-leaked lines
* fix(ping): fix compile error
* fix!(ping): change the receiving target from server to topic
* refactor(ping): change class name from `Service` to `Server`
* fix(interpreter): fix compile error
* chore(ping): fix some errors
* refactor(ping): rename package from `ros_ping_with_service` to `ros_ping`
* refactor(ping): move service class to `common` directory
* refactor(ping): add `const` keyword
* refactor(ping): change name
* Merge remote-tracking branch 'origin/master' into feat/heat_beat
* chore: fix compile errors
* Merge pull request `#913 <https://github.com/tier4/scenario_simulator_v2/issues/913>`_ from tier4/use/autoware_github_actions
* doc(spell-check): give reasons for ignoring misspellings in comment
* chore(spell-check): ignore error from "euclidian"
* fix(typo): defintiion => definition
* fix(typo): refenrece => reference
* refactor(heartbeat): fix enum class to snake_case
* fix(heartbeat): fix deadlock
* fix(heartbeat):
* feature(heartbeat): add timeout to heartbeat_checker_node
* feat(interpreter): add heartbeat_checker_node
* Merge pull request `#903 <https://github.com/tier4/scenario_simulator_v2/issues/903>`_ from tier4/feature/empty/parameter_value_distribution
* doc(interpreter): add OpenSCENARIO version to xsd descriptions
* Merge remote-tracking branch 'origin/master' into feature/empty/parameter_value_distribution
* Fix scenario loading error
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Fix scenario loading path
* Merge pull request `#883 <https://github.com/tier4/scenario_simulator_v2/issues/883>`_ from tier4/feature/interpreter/priority
* Rename member function `Maneuver::override_events` to `overrideEvents`
* Update `CustomCommand::start` to return `void` instead of `int`
* Lipsticks
* Rename `ICustomCommand` to `CustomCommand`
* Lipsticks
* Merge pull request `#900 <https://github.com/tier4/scenario_simulator_v2/issues/900>`_ from tier4/feature/traffic_simulator/behavior-parameter
* add a part for checking priority features to all-in-one.yaml
* add DummyLongRunningAction and support skip priority
* Add new message type `traffic_simulator_msgs::msg::DynamicConstraints`
* Merge branch 'fix/interpreter/custom_command_action' into feature/interpreter/priority
* Merge branch 'master' into fix/interpreter/custom_command_action
* rename base -> entity
* fix CustomCommand
* Update syntax `SpeedProfileAction` to call `API::setBehaviorParameter`
* Add some max constraints to `msg::BehaviorParameter`
* Update `Performance::operator traffic_simulator_msgs::msg::Performance`
* Update syntax `Performance` to version 1.2
* Merge branch 'master' into feature/bt_auto_ros_ports
* Merge pull request `#898 <https://github.com/tier4/scenario_simulator_v2/issues/898>`_ from tier4/feature/interpreter/speed-profile-action
* Add test scenario `LongitudinalAction.SpeedProfileAction`
* Rename `DriverModel` to `BehaviorParameter`
* Delete distribution evaluation implementation
* Clanup member function `SpeedProfileAction::apply`
* Lipsticks
* Delete distribution evaluation implementation
* Add new member function `SpeedProfileAction::apply`
* Update 'SpeedProfileAction' to respect attribute 'entityRef'
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* add CustomCommandActionBase class
* Update syntax `SpeedProfileAction` to request speed change sequentially as given `SpeedProfileEntry`
* Fix cardinality of element `SpeedProfileEntry` of syntax `SpeedProfileEntry`
* Add new syntax `SpeedProfileEntry`
* Add new syntax `DynamicConstraints`
* Add new syntax `SpeedProfileAction`
* Update syntax `LongitudinalAction` to version 1.2
* Merge pull request `#896 <https://github.com/tier4/scenario_simulator_v2/issues/896>`_ from tier4/refactor/traffic_simulator/spawn
* Update openscenario_interpreter description
* Merge remote-tracking branch 'origin/master' into feature/interpreter/priority
* fix override
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Extract openscenario_preprocessor
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/spawn
* Update API::spawn (VehicleEntity) to receive position
* Refactor
* Add TODO comments
* Update `API::spawn` (PedestrianEntity) to receive position
* Merge remote-tracking branch 'origin/master' into feature/parameter_value_distribution
* Update `API::spawn` (MiscObjectEntity) to receive position
* Merge pull request `#893 <https://github.com/tier4/scenario_simulator_v2/issues/893>`_ from tier4/feature/interpreter/follow-trajectory-action-3
* Merge branch 'master' into feature/interpreter/follow-trajectory-action-3
* Merge pull request `#890 <https://github.com/tier4/scenario_simulator_v2/issues/890>`_ from tier4/refactor/test_runner
* Merge remote-tracking branch 'origin/master' into fix/shifted_bounding_box
* Update data member `EntityBase::stand_still_duration\_` to not to be optional
* Remove unused argument from member function `CatalogReference::make`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/follow-trajectory-action-3
* Merge remote-tracking branch 'origin/master' into refactor/test_runner
* Merge pull request `#892 <https://github.com/tier4/scenario_simulator_v2/issues/892>`_ from tier4/feature/interpreter/follow-trajectory-action-2
* Apply ament_clang_format
* Fix bug in preprocessor/derive server
* Integrate preprocessor into scenario_test_runner.py
* Add openscenario_preprocessor to launch file
* Confirm operation check for preprocessor
* Add annotations to avoid compiler warnings
* Add new test scenario `RoutingAction.FollowTrajectoryAction.yaml`
* Add new syntax class `Vertex`
* Add new syntax class `Polyline`
* Add new syntax class `Shape`
* Add new syntax class `Trajectory`
* Add new syntax class `TrajectoryRef`
* Implement minimal interface of Preprocessor
* Add new syntax class `FollowingMode`
* Add new syntax class `TrajectoryFollowingMode`
* Add new syntax class `Timing`
* Add new syntax class `None`
* Add new syntax class `TimeReference`
* Add new syntax class `FollowTrajectoryAction`
* Merge pull request `#891 <https://github.com/tier4/scenario_simulator_v2/issues/891>`_ from tier4/feature/interpreter/follow-trajectory-action
* Merge pull request `#877 <https://github.com/tier4/scenario_simulator_v2/issues/877>`_ from tier4/feature/interpreter/printParameter
* Move Preprocessor class implementation to cpp file
* Add openscenario_preprocessor_node
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Move namespace `lane_change` into new header `data_type/lane_change.hpp`
* Add openscenario_preprocessor.hpp
* Move namespace `speed_change` into new header `data_type/speed_change.hpp`
* Lipsticks
* Cleanup struct `Constraint` and `RelativeTargetSpeed`
* Lipsticks
* Add empty Preprocessor class
* Apply ament_clang_format
* Implement ProbabilityDistributionSet
* Add nodiscard annotation
* Apply ament_clang_format
* Implement Histogram
* Implement PoissonDistribution
* Implement UniformDistribution
* Move StochasticDistributionClass to distribution.hpp
* Implement normal_distribution
* accept all priority values
* Merge branch 'master' into feature/interpreter/priority
* Merge pull request `#881 <https://github.com/tier4/scenario_simulator_v2/issues/881>`_ from tier4/fix/interpreter/on-activate
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Add new member function `Interpreter::reset`
* Fix build errors
* Fix exception handler of phase `on_activate` to return `FAILURE`
* Add implementation hints to stochastic distributions
* Cleanup
* Fix XSD annotations
* Apply ament_clang_format
* Fix build errors
* Add readGroups function
* Splice `initialize_storyboard` to its caller
* Fix build errors
* Implement Stochastic::Stochastic
* Add missing attribute to StochasticDistribution
* Add PoissonDistribution
* Add UniformDistribution
* Add NormalDistribution
* Add HistogramBin
* Add Histogram
* Fix Class name
* Add ProbabilityDistributionSetElement
* Add ProbabilityDistribution
* Add StochasticDistributionType
* Add StochasticDistribution
* Add UserDefinedDistribution
* Add DistributionSetElement
* Add DistributionSet
* Add Range
* Add DistributionRange
* Add ParameterValueSet
* Add ValueSetDistribution
* Add DeterministicSingleParameterDistributionType
* Add DeterministicMultiParameterDistributionType
* Fix Deterministic
* Fix xsd annotation
* Add DeterministicSingleParameterDistribution
* Add DeterministicMultiParameterDistribution
* Add DeterministicParameterDistribution
* Add Deterministic
* Fix implementation of DistributionDefinition
* Merge remote-tracking branch 'origin/master' into feature/concealer/acceleration
* Merge pull request `#823 <https://github.com/tier4/scenario_simulator_v2/issues/823>`_ from tier4/feature/start_npc_logic_api
* Fix implementation of ParameterValueDistribution
* fix function decl/impl order
* ament_clang_format
* add a new custom command (printParameter)
* Add Stochastic
* Add Deterministic
* Add DistributionDefinition
* Add ParameterValueDistribution
* Update OpenScenarioCategory to OSC 1.2
* ament_clang_format
* add member functions for priority to Maneuver
* add std::flush to test command
* move some StoryboardElement's members from public to protected
* Lipsticks
* Rename member function `Autoware::ready` to `Autoware::engaged`
* Merge remote-tracking branch 'origin/master' into fix/service-request-until-success
* Remove member function `Storyboard::start() -> void`
* Merge remote-tracking branch 'origin/master' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge branch 'master' into fix/simple_sensor_simulator/fast_occupancy_grid
* Merge remote-tracking branch 'origin/master' into fix/ci_catch_rosdep_error
* Merge remote-tracking branch 'origin/fix/ci_error' into feature/start_npc_logic_api
* Merge branch 'master' into feature/occupancy_grid_docs
* Remove unused member function `AutowareUniverse::isReady`
* Update `Interpreter` to set `Configuration::initialize_duration` to zero
* Lipsticks
* Update `Interpreter` to start non-ego entities at Autoware reaches `DRIVING` state.
* Merge remote-tracking branch 'origin/refactor/interpreter/scope' into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/start_npc_logic_api
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Shota Minami, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.6 (2022-08-30)
------------------
* Merge pull request `#852 <https://github.com/tier4/scenario_simulator_v2/issues/852>`_ from tier4/refactor/catalog
* Refactor
* Fix runtime error
* Fix build error
* Refactor
* Refactor & Apply linter
* Clean code
* Add debug messages
* Refactor CatalogReference
* Fix build errors
* Fix build errors
* Extract CatalogReference::make function
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge pull request `#847 <https://github.com/tier4/scenario_simulator_v2/issues/847>`_ from tier4/feature/value_constraint
* Revert parameter_declaration.cpp
* Revert parameter_declaration.cpp
* Replace "Tier IV" with "TIER IV"
* Refactor
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Merge remote-tracking branch 'origin/master' into feature/value_constraint
* Apply linter
* Refactor
* Revert ParameterCondition::compare
* Refactor
* Refactor
* Apply clang-format
* Fix logic miss
* Merge pull request `#846 <https://github.com/tier4/scenario_simulator_v2/issues/846>`_ from tier4/refactor/interpreter/scope
* Update `VehicleCategory` to accept any VehicleCategory identifier OpenSCENARIO specified.
* Move `EntityObject` implementation into .cpp file
* Rename header file `openscenario.hpp` to `open_scenario.hpp`
* Rename struct `Scope::GlobalEnvironment` to `ScenarioDefinition`
* Apply clang-format
* Implement evaluate functions of value constraints
* Fix structure of ParameterDeclaration
* Implement loading part of ValueConstraintGroup
* Cleanup
* Move data member `Scope::pathname` into struct `OpenScenario`
* Move some member function of `GlobalEnvironment` into `Entities`
* Update `GlobalEnvironment` to hold pointer to `Entities`
* Add value_constraint codes
* Add constraint group to ParameterDeclaration
* Extract compare function to a single file
* Move compare function
* Merge remote-tracking branch 'tier/master' into fix/concealer-dangling-reference
* Merge remote-tracking branch 'tier/master' into fix/obstacle_detection_raycaster
* Merge remote-tracking branch 'origin/master' into update/rviz_display
* Merge branch 'master' into fix/trajectory_offset
* Merge pull request `#801 <https://github.com/tier4/scenario_simulator_v2/issues/801>`_ from tier4/feature/openscenario/non_instantaneous_actions
* Merge pull request `#830 <https://github.com/tier4/scenario_simulator_v2/issues/830>`_ from tier4/feature/interpreter/relative-heading-condition
* Merge remote-tracking branch 'origin/master' into fix/interpreter/transition_assertion
* Add new NonStandardOperation `evaluateRelativeHeading`
* Update `UserDefinedValueCondition` to support function-style expression (EXPERIMENTAL)
* Delete unused variable
* Merge remote-tracking branch 'origin/master' into feature/interpreter/relative-heading-condition
* Add new header `regex/function_call_expression.hpp`
* Merge pull request `#829 <https://github.com/tier4/scenario_simulator_v2/issues/829>`_ from tier4/improve/exclude-too-large-topic-from-ros2-bag-record
* Update `ros2 bag record` to exclude a large topic
* Merge pull request `#818 <https://github.com/tier4/scenario_simulator_v2/issues/818>`_ from tier4/feature/autoware/request-to-cooperate
* Implement Action::endsImmediately
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Lipsticks
* Rename `CooperatePolicy` to `Cooperator`
* Fix foolish bugs ,and I hate myself for being so useless
* Fix logic miss in InitActions::endsImmediately
* Fix typos
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Add new experimental Controller property `cooperatePolicy`
* Merge remote-tracking branch 'origin/master' into fix/stop_position
* Merge pull request `#821 <https://github.com/tier4/scenario_simulator_v2/issues/821>`_ from tier4/feature/linelint
* remove empty file
* Fix runtime error
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Demangle type names
* Demangle type names
* Fix compile errors and linter errors
* Implement Init class inferfaces
* Implement interfaces of InitAction class
* Implement InitActions::startInstantaneousActions
* Implement accomplished functions
* Divide interface of Private class
* inherited StoryboardElement at InitActions class
* inherited StoryboardElement at InitActions class
* Replace all_elements with 3 arrays
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge pull request `#805 <https://github.com/tier4/scenario_simulator_v2/issues/805>`_ from tier4/doc/4th-improvement
* Merge remote-tracking branch 'origin/master' into feature/autoware/request-to-cooperate
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Merge pull request `#783 <https://github.com/tier4/scenario_simulator_v2/issues/783>`_ from tier4/refactor/interpreter/simulator-core
* Remove `src/procedure.cpp` from `CMakeLists.txt`
* Update ReleaseNotes
* Fix spells
* Merge https://github.com/tier4/scenario_simulator_v2 into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Update `SimulatorCore::core` to be private
* Rename struct `GeneralCommand` to `CoordinateSystemConversion`
* Move `getBoundingBoxDistance` into `SimulatorCore::ConditionEvaluation`
* Move `getLongitudinalDistance` into `SimulatorCore::GeneralCommand`
* Move `getRelativePose` into `SimulatorCore::GeneralCommand`
* Rename header `procedure.hpp` to `simulator_core.hpp`
* Move `getTrafficRelationReferees` into `NonStandardOperation`
* Fix `activatePerformanceAssertion` to check if `Controller` specified
* Merge remote-tracking branch 'origin/master' into feature/openscenario/non_instantaneous_actions
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Cleanup geometry type conversions
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_relative_pose_with_lane_pose
* Refine implementation
* Divide by instantaneous on load
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Add new function `activatePerformanceAssertion` as `NonStandardOperation`
* Remove member function `ScenarioObject::activateSensors`
* Add new struct `SimulatorCore::NonStandardOperation`
* Lipsticks
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'feature/get_distance_to_lane_bound' of https://github.com/tier4/scenario_simulator.auto into feature/geometry_lib
* Merge remote-tracking branch 'origin/master' into doc/4th-improvement
* Fix runtime error
* Fix compile errors
* Implement non instantaneous init action move
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Merge branch 'master' into fix/interpreter/transition_assertion
* Move some free functions into struct `SimulatorCore`
* Remove member function `Controller::assign` and `ObjectController::assign`
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/simulator-core
* Move some free functions into struct `SimulatorCore`
* Move free function `toLanePosition` into struct `SimulatorCore::GeneralCommand`
* Move free function `getEntityStatus` into struct `SimulatorCore::ConditionEvaluation`
* Move free function `addMetric` into struct `SimulatorCore::GeneralCommands`
* Move variable `connection` into new struct `SimulatorCore`
* Contributors: Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.5 (2022-06-16)
------------------
* Merge pull request `#813 <https://github.com/tier4/scenario_simulator_v2/issues/813>`_ from tier4/fix/boost_depend
* add boost to the depends of concealer and interpreter
* Merge pull request `#793 <https://github.com/tier4/scenario_simulator_v2/issues/793>`_ from tier4/fix/build-error-humble
* Merge remote-tracking branch 'origin/master' into feature/change_engage_api_name
* fix(openscenario_interpreter): modify build error in both galactic and humble
* more build fixes
* is console lambda inline
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/get_distance_to_lane_bound
* Merge pull request `#796 <https://github.com/tier4/scenario_simulator_v2/issues/796>`_ from tier4/refactor/concealer/virtual-functions
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pull_over_metrics
* Fix code style divergences
* Merge branch 'master' into feature/change_engage_api_name
* Fix local function `everyone_engageable`
* Merge remote-tracking branch 'origin/master' into refactor/concealer/virtual-functions
* Remove member function `API::engage` and `API::ready`
* Remove some member functions for Autoware.Universe from API
* Merge pull request `#797 <https://github.com/tier4/scenario_simulator_v2/issues/797>`_ from tier4/feature/occupancy_grid_sensor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge pull request `#791 <https://github.com/tier4/scenario_simulator_v2/issues/791>`_ from tier4/doc/arrange_docs_and_fix_copyright
* Merge branch 'master' into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/arrange_docs_and_fix_copyright
* Fix Licence
* Merge pull request `#774 <https://github.com/tier4/scenario_simulator_v2/issues/774>`_ from tier4/feature/allow_event_starttriger_ommision
* Merge remote-tracking branch 'origin/feature/allow_event_starttriger_ommision' into feature/allow_event_starttriger_ommision
* Refactor
* Merge remote-tracking branch 'origin/master' into fix/autoware/reverse-gear
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Refactor
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge pull request `#758 <https://github.com/tier4/scenario_simulator_v2/issues/758>`_ from tier4/feature/interpreter/instantaneously-transition
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into build/add_missing_depend
* Fix runtime errors & Delete debug codes
* Merge remote-tracking branch 'origin/master' into feature/allow_event_starttriger_ommision
* Merge remote-tracking branch 'origin/master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into feature/zmqpp_vendor
* Merge pull request `#785 <https://github.com/tier4/scenario_simulator_v2/issues/785>`_ from tier4/doc/improve
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/occupancy_grid_sensor
* Merge remote-tracking branch 'origin/master' into doc/improve
* Update `Storyboard` to call thunks after parsing
* Fix old "TierIV" annotation
* Fix StoryboardElementStateCondition registration to perform on `Storyboard` startTransition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Merge pull request `#777 <https://github.com/tier4/scenario_simulator_v2/issues/777>`_ from tier4/feature/indicator_condition
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* fix problems in server
* WIP : Debug
* Update interpreter to publish context topic once before deactivation
* Refactor
* Rename **StateString to **StateName
* Fix compile errors
* Implement TurnIndicatorsState as an UserDefinedValueCondition
* Apply linter
* Fix compile error
* Change implementation of readElement
* Merge remote-tracking branch 'origin/master' into feature/interpreter/instantaneously-transition
* Make the event starttrager optional and set default return true
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Merge pull request `#760 <https://github.com/tier4/scenario_simulator_v2/issues/760>`_ from tier4/feature/emergency_state_for_fault_injection
* Fix assertion
* Update ReleaseNotes
* Add new member function `Interpreter::withTimeoutHandler` and `defaultTimeoutHandler`
* Update `Storyboard` to engage Autoware on `startTransition`
* Update `Interpreter` to not to evaluate `Storyboard` if simulation-time < 0
* Add currentEmergencyState to UserDefinedValueCondition as a value source
* Fix regex
* Move `updateFrame` call into `Interpreter` from `ScenarioDefinition`
* Remove header `utility/pair.hpp`
* Update `Storyboard.Init` to evaluate during the initialization phase of the interpreter
* Lipsticks
* Update member function `StoryboardElement::notify` to `transitionTo`
* Update `StoryboardElement::evaluate` not to stay in fooTransition state
* Lipsticks
* Add experimental data member `StoryboardElement::callbacks`
* Merge branch 'master' into AJD-331-optimization-2nd-stage
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'master' into fix/interpreter/missing_autoware_launch
* Merge branch 'tier4:master' into AJD-331-optimization-2nd-stage
* Contributors: Adam Krawczyk, Daisuke Nishimatsu, Daniel Marczak, Kotaro Yoshimoto, Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, Tatsuya Yamasaki, Yuma Nihei, kyabe2718, yamacir-kit

0.6.4 (2022-04-26)
------------------
* Merge remote-tracking branch 'origin/master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge pull request `#740 <https://github.com/tier4/scenario_simulator_v2/issues/740>`_ from tier4/refactor/traffic_simulator/traffic_light_manager
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/reset_acecel_in_request_speed_change
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Rename member function `getTrafficRelation` to `getTrafficRelationReferees`
* Add new member function `TrafficLight::getTrafficRelation`
* Add new member function `TrafficSignal::set`
* Merge pull request `#731 <https://github.com/tier4/scenario_simulator_v2/issues/731>`_ from tier4/feature/interpreter/category
* Merge remote-tracking branch 'origin/master' into refactor/traffic_simulator/traffic_light_manager
* Remove struct `Color` and `Arrow` from interpreter
* Switch struct `TrafficLight` to experimental version
* Merge pull request `#728 <https://github.com/tier4/scenario_simulator_v2/issues/728>`_ from tier4/fix/interpreter/interrupt
* Remove enumeration `TrafficLightColor::NONE`
* Merge branch 'master' into fix/interpreter/interrupt
* Remove member function `TrafficLightManager::get(Arrow|Color)`
* Merge branch 'tier4:master' into feature/awf_universe_instruction
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'tier4:master' into AJD-331-make-zmq-client-work-through-network
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#735 <https://github.com/tier4/scenario_simulator_v2/issues/735>`_ from tier4/feature/interpreter/object-controller
* Fix `ScenarioObject::activateSensors` to check if `Controller` was given
* Support new controller property `isClairvoyant`
* Remove `Controller::operator []` and `Properties::operator []`
* Add new member function `Properties::get<T>`
* Update `ScenarioObject::activateSensors` to make `DetectionSensorConfiguration` explicitly
* Merge branch 'master' into fix/interpreter/interrupt
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/improve_ego_lane_matching
* Merge pull request `#734 <https://github.com/tier4/scenario_simulator_v2/issues/734>`_ from tier4/fix/interpreter/global-action
* Update `record::start` to redirect child process stdout to `/dev/null`
* Remove unnecessary prints from the interpreter and simulator
* Update Interpreter to catch error on actionvation
* Update `record::stop` wait 3 seconds before send `SIGINT`
* Update syntax `UserDefinedAction` to callable from `InitActions`
* Cleanup `Private::evaluate`
* Fix syntax `GlobalAction` to be callable from `InitActions`
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/waypoint_height
* Update syntax `Vehicle`, `Pedestrian` and `MiscObject` to set subtype correctly
* Update syntax `MiscObjectCategory` to support `operator EntitySubtype`
* Update syntax `PedestrianCategory` to support `operator EntitySubtype`
* Update syntax `VehicleCategory` to support `operator EntitySubtype`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Merge pull request `#727 <https://github.com/tier4/scenario_simulator_v2/issues/727>`_ from tier4/feature/interpreter/reader
* Merge branch 'master' into feature/interpreter/reader
* Merge pull request `#726 <https://github.com/tier4/scenario_simulator_v2/issues/726>`_ from tier4/feature/semantics
* remove unused comments
* rename data field
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/ignore_raycast_result
* Add new experimental substitution `$(ros2 ...)`
* set default value fron interpreter
* Lipsticks
* Cleanup free function `readAttribute`
* Remove macro `OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK`
* Cleanup free function `substitute`
* add disconect() to ~Interpreter(). stop zeromq call if shut down.
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/semantics
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into doc/release_note_format
* Merge pull request `#711 <https://github.com/tier4/scenario_simulator_v2/issues/711>`_ from tier4/refactor/interpreter/storyboard-element
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Update `StoryboardElement::start` to not to pure-virtual
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/semantics
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into doc/release_note_format
* Merge pull request `#720 <https://github.com/tier4/scenario_simulator_v2/issues/720>`_ from tier4/refactor/interpreter/execution
* Merge branch 'tier4:master' into AJD-345-random_test_runner_with_autoware_universe
* enable pass copile in all packages
* Cleanup
* Update main loop to not to stop even if storyboard was completed
* Lipsticks
* Update `ExecutionTimer::invoke` to receive thunk
* Lipsticks
* Remove member function `StoryboardElement::ready`
* Add new data member `StoryboardElement::start_trigger`
* Merge branch 'master' into AJD-331-optimization
* Remove member function `Storyboard::stop`
* Add new data member `StoryboardElement::stop_trigger`
* Update syntax `Trigger` to be default constructible
* Update `StoryboardElement::stop` to not to pure-virtual
* Update `StoryboardElement::run` to not to pure-virtual
* Update `StoryboardElement::accomplished` to not to pure-virutal
* Remove free function `readElementsAsElement`
* Rename free function `callWithElements` to `traverse`
* Update free function `callWithElements` template parameter
* Add new data member `StoryboardElement::elements`
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element
* Remove obsoleted `StoryboardElement`'s member functions
* Add new member function `StoryboardElement::is<STATE>`
* Merge branch 'tier4:master' into AJD-331-optimization
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Merge branch 'master' into AJD-345-random_test_runner_with_autoware_universe
* Contributors: Daniel Marczak, Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, kyabe2718, yamacir-kit

0.6.3 (2022-03-09)
------------------
* Merge pull request `#713 <https://github.com/tier4/scenario_simulator_v2/issues/713>`_ from tier4/fix/interpreter/traffic-signal-state-action
  Fix/interpreter/traffic signal state action
* Update syntax `TrafficSignalState` to reset both of color and arrow if `none` specified
* Fix `TrafficSignalStateAction::start` to not to throw error if given state is valid
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/waypoint_offset
* Merge pull request `#704 <https://github.com/tier4/scenario_simulator_v2/issues/704>`_ from tier4/feature/autoware-external-api
  Feature/autoware external api
* Rename member function `setUpperBoundSpeed` to `setVelocityLimit`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/idead_steer_acc_geard
* Merge pull request `#696 <https://github.com/tier4/scenario_simulator_v2/issues/696>`_ from tier4/dependency/remove-autoware-auto
  Dependency/remove autoware auto
* Merge pull request `#663 <https://github.com/tier4/scenario_simulator_v2/issues/663>`_ from tier4/dependency/remove-architecture-proposal
  Dependency/remove architecture proposal
* Merge remote-tracking branch 'origin/dependency/remove-architecture-proposal' into dependency/remove-autoware-auto
* Remove macro identifier `SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO`
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#693 <https://github.com/tier4/scenario_simulator_v2/issues/693>`_ from tier4/refactor/interpreter/storyboard-element-state
  Refactor/interpreter/storyboard element state
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/storyboard-element-state
* Fix simulation context output
* Rename member function `StoryboardElement::currentState` to `state`
* Merge pull request `#692 <https://github.com/tier4/scenario_simulator_v2/issues/692>`_ from tier4/refactor/interpreter/storyboard-element
  Refactor/interpreter/storyboard element
* Update member function `Scope::ref<T>` to allow polymorphic type
* Remove obsoleted metafunction headers
* Remove member function `Expression::currentState`
* Remove meaningless semicolon
* Lipsticks
* Lipsticks
* Update the immediate actions to effect at the timing of `startTransition`
* Merge pull request `#688 <https://github.com/tier4/scenario_simulator_v2/issues/688>`_ from tier4/fix/traffic_simulator/traffic_light_manager
  Fix/traffic simulator/traffic light manager
* Lipsticks
* Merge pull request `#681 <https://github.com/tier4/scenario_simulator_v2/issues/681>`_ from tier4/refactor/interpreter/storyboard-element
  Refactor/interpreter/storyboard element
* Add pure virtual function `StoryboardElement::elements`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/zeromq_multi_client
* Merge pull request `#682 <https://github.com/tier4/scenario_simulator_v2/issues/682>`_ from tier4/fix/interpreter/explicit_nop
  explicit ignore ':'
* Update syntax `StoryboardElement` to be virtual class
* avoid early return
* Merge branch 'fix/interpreter/explicit_nop' of https://github.com/tier4/scenario_simulator_v2 into feature/zeromq_multi_client
* explicit ignore ':'
* Remove member function `StoryboardElement::changeStateIf`
* Lipsticks
* Merge pull request `#679 <https://github.com/tier4/scenario_simulator_v2/issues/679>`_ from tier4/refactor/interpreter/scope
  Refactor/interpreter/scope
* Rename member function `resolveFrontPrefix` to `resolvePrefix`
* Move struct `is` into header `object.hpp`
* Update some Action/Condtions to use `Scope::ref<T>`
* Update `Scope::ref` to receive return value type as template parameter
* Update `EnvironmentFrame::find<T>` to return T type value
* Move member function `EnvironmentFrame::lookdown` into prefixless`find`
* Lipsticks
* Simplify member function `EnvironmentFrame::lookdown`
* Rename `EnvironmentFrame::frames` to `resolveFrontPrefix`
* Update some `EnvironmentFrame`'s member functions to receive `Prefixed<Name>`
* Move some member functions into header
* Simplify member function `EnvironmentFrame::outermostFrame`
* Rename member function `findObject` to `ref`
* Move member function `findObject` implemenation into header
* Lipsticks
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge pull request `#678 <https://github.com/tier4/scenario_simulator_v2/issues/678>`_ from tier4/refactor/interpreter
  Refactor/interpreter
* Remove data member `current_scenario`
* Lipsticks
* Merge pull request `#672 <https://github.com/tier4/scenario_simulator_v2/issues/672>`_ from tier4/fix/interpreter/lifecycle
  Fix/interpreter/lifecycle
* Merge branch 'master' into fix/interpreter/lifecycle
* ament_clang_format
* construct/destruct connection at on_activate/on_deactivate
* fix missing #include
* make connection std::unique_ptr
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Merge remote-tracking branch 'origin/master' into dependency/remove-architecture-proposal
* Remove package `autoware_api_msgs` from dependency
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.2 (2022-01-20)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/lane_change_trajectory_shape
* Merge pull request `#670 <https://github.com/tier4/scenario_simulator_v2/issues/670>`_ from tier4/feature/request_speed_change_in_pedestrian
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change_in_pedestrian
* Merge branch 'master' into feature/request_speed_change_in_pedestrian
* Merge pull request `#668 <https://github.com/tier4/scenario_simulator_v2/issues/668>`_ from tier4/feature/interpreter/lane-change-action
* Update member function `LaneChangeAction::accomplished`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/lane-change-action
* Merge pull request `#669 <https://github.com/tier4/scenario_simulator_v2/issues/669>`_ from tier4/refactor/add_speed_change_namespace
* remove old API
* add speed_change namespace
* Fix syntax `Orientation` to be default constructible
* Lipsticks
* Add new test scenario `LateralAction.LaneChangeAction`
* Update some syntaxes to support conversion operator
* Update syntax `LaneChangeAction` to use `API::requestLaneChange` (experimental)
* Merge https://github.com/tier4/scenario_simulator.auto into feature/control_from_relation_id
* Merge pull request `#665 <https://github.com/tier4/scenario_simulator_v2/issues/665>`_ from tier4/feature/interpreter/speed-action
* Update member function `SpeedAction::accomplished`
* Update some structures to support cast operator
* Lipsticks
* Remove obsoleted code
* Remove member function `SpeedAction::reset`
* Update syntax `SpeedAction` to use new API `requestSpeedChange`
* Update `EgoEntity` to override `EntityBase::requestSpeedChange`
* Add conversion operators to some OSC structures
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/lane_change_trajectory_shape
* Merge remote-tracking branch 'origin/master' into feature/traffic_simulator/vehicle_model
* Merge pull request `#581 <https://github.com/tier4/scenario_simulator_v2/issues/581>`_ from Utaro-M/matsuura/feature/add-time-to-panel
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
* add simulation time to panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.6.1 (2022-01-11)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/get_driver_model_in_pedestrian
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/request_speed_change
* Merge pull request `#649 <https://github.com/tier4/scenario_simulator_v2/issues/649>`_ from tier4/fix/interpreter/controller
* Add some member functions to class `Properties`
* Fix `Controller::assign` to not to overwrite parameter `see_around`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge pull request `#628 <https://github.com/tier4/scenario_simulator_v2/issues/628>`_ from tier4/feature/avoid_overwrite_acceleration
* Merge pull request `#641 <https://github.com/tier4/scenario_simulator_v2/issues/641>`_ from tier4/feature/interpreter/expr
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/avoid_overwrite_acceleration
* add copyright
* fix variant
* ament_clang_format
* Merge branch 'master' into feature/interpreter/expr
* remove debug messages
* add OSC expression evaluator
* Remove member function `Controller::defaultDriverModel`
* Update `Controller::assign` to use `API::getDriverModel`
* Remove cast operator `Controller::operator DriverModel`
* Remove cast operator `ObjectController::operator DriverModel`
* Merge branch 'feature/avoid_overwrite_acceleration' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/set_acceleration_deceleration
* Add new member function `Controller::makeDefaultDriverModel`
* Merge remote-tracking branch 'origin/master' into feature/avoid_overwrite_acceleration
* Comment-out changes
* getDriverModel API first.
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.6.0 (2021-12-16)
------------------
* Merge pull request `#614 <https://github.com/tier4/scenario_simulator_v2/issues/614>`_ from tier4/use-autoware-auto-msgs
* Fix `TransitionAssertion` to stop if class `Autoware` down
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/pass_goal_poses_to_the_plugin
* Update CMakeLists to not to reference undefined variable
* Update packages to compile with `awf/autoware_auto_msgs` if flag given
* Update `API::attachDetectionSensor` to detect Autoware architecture
* Update `API::attachLidarSensor` to detect Autoware architecture
* Merge remote-tracking branch 'origin/master' into use-autoware-auto-msgs
* Merge pull request `#617 <https://github.com/tier4/scenario_simulator_v2/issues/617>`_ from tier4/autoware-universe-concealer
* fix no ground topic name for tier4/proposal
* change no_ground pointcloud topic name
* some changes to run psim with autoware_universe
* Contributors: MasayaKataoka, Takayuki Murooka, Tatsuya Yamasaki, yamacir-kit

0.5.8 (2021-12-13)
------------------
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge pull request `#612 <https://github.com/tier4/scenario_simulator_v2/issues/612>`_ from tier4/feature/remove_newton_method_from_get_s_value
* remove todo comment
* Merge remote-tracking branch 'tier/master' into feature/AJD-288-AAP_with_scenario_simulator_instruction
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_newton_method_from_get_s_value
* apply reformat
* remove default argument
* Merge pull request `#605 <https://github.com/tier4/scenario_simulator_v2/issues/605>`_ from tier4/refactor/interpreter/reference
* Fix typo
* Rename `Prefixed::fully_prefixed` to `Prefixed::absolute`
* Add new member function `EnvironmentFrame::outermostFrame`
* Rename member function `lookupQualifiedElement` to `find`
* Update `lookupQualifiedElement` to receive `PrefixedName`
* Update `PrefixedName::prefixes.front()` to not to store empty string
* Update `lookupQualifiedElement` to be non-static member function
* Update `lookupQualifiedElement` argument iterators to not to include variable name
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/reference
* Update `EnvironmentFrame::lookupUnqualifiedScope` to not to return nullptr
* Rename member function `lookupChildScope` to `frames`
* Lipsticks
* Remove deprecated member function `EnvironmentFrame::fullyQualifiedName`
* Rename data member `anonymous_children` to `unnamed_inner_frames`
* Rename data member `parent` to `outer_frame`
* Rename member function `lookupUnqualifiedElement` to `lookup`
* Rename header `identifier.hpp` to `name.hpp`
* Rename some typenames
* Add new type `QualifiedIdentifier`
* Add new type `UnqualifiedIdentifier`
* Rename `EnvironmentFrame::scope_name` to `EnvironmentFrame::qualifier`
* Remove `Scope`'s private constructor
* Remove member function `Scope::makeChildScope`
* Update member function `Scope::makeChildScope` to be private
* Lipsticks
* Contributors: Masaya Kataoka, MasayaKataoka, Piotr Zyskowski, yamacir-kit

0.5.7 (2021-11-09)
------------------
* Merge pull request `#604 <https://github.com/tier4/scenario_simulator_v2/issues/604>`_ from tier4/refactor/interpreter/function-name
* Remove type alias `XML`
* Merge https://github.com/tier4/scenario_simulator_v2 into fix/get_s_value
* Rename type `Element` to `Object`
* Rename member function `Scope::localScope` to `Scope::local`
* Rename data member `last_checked_values` to `results`
* Remove alias template `IsOptionalElement`
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/function-name
* Merge pull request `#579 <https://github.com/tier4/scenario_simulator_v2/issues/579>`_ from tier4/feature/interpreter/catalog
* Remove header `utility/pugi_extension.hpp`
* Remove header `utility/indent.hpp`
* Replace `std::empty(x)` to `x.empty()`
* Lipsticks
* Lipsticks
* Merge pull request `#597 <https://github.com/tier4/scenario_simulator_v2/issues/597>`_ from tier4/refactor/traffic_simulator/spawning
* Fix function name to lowerCamelCase from snake_case
* Lipsticks
* Lipsticks
* Replace flag `is_ego` to string typed plugin name
* fix invoking yaml2xoc
* invoke yaml2xosc using python3 command
* Merge branch 'master' into feature/interpreter/catalog
* Update `API::spawn` argument order
* Remove meaningless argument `is_ego` from some `spawn` overloads
* Update `API::spawn` to not to apply `setEntityStatus` to rest arguments
* Update `AddEntityAction::operator ()` to use `TeleportAction::teleport`
* Add new member function `teleport` to class `TeleportAction`
* apply() dispatches using is_also<T> instead of is<T>
* fix binder
* return former folder
* remove fold expression
* support foxy
* fix trivial bug
* ament_clang_format
* add catalog test
* add CatalogReference
* Merge branch 'master' into feature/interpreter/catalog
* catalog parameter
* Merge branch 'master' into feature/interpreter/catalog
* basic impl
* convert scenario file from yaml to xosc
* add catalog_reference.hpp
* Merge branch 'master' into feature/interpreter/catalog
* Merge branch 'master' into feature/interpreter/catalog
* add Catalog
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.5.6 (2021-10-28)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2
* Merge branch 'tier4:master' into matsuura/feature/add-icon-to-panel
* Merge pull request `#567 <https://github.com/tier4/scenario_simulator_v2/issues/567>`_ from tier4/feature/interpreter/user-defined-value-condition
* Move some messages into new package `openscenario_msgs`
* Fix `UserDefinedValueCondition` to support to receive multiple message
* Rename UserDefinedValueCondition example node to `count_up`
* Update `UserDefinedValueCondition` to return false if no message received
* Add static member function `ParameterCondition::compare`
* Add new static member function `ParameterSetAction::set`
* Update `ParameterDeclaration`'s from message constructor to receive `Scope`
* Update `ParameterType` and `ParameterDeclaration` constructor
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_debug_marker
* Merge remote-tracking branch 'origin/master' into feature/interpreter/user-defined-value-condition
* Fix `MagicSubscription` to not to copy construct
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/cleanup_logger
* Update `UserDefinedValueCondition` to receive message
* Merge pull request `#571 <https://github.com/tier4/scenario_simulator_v2/issues/571>`_ from tier4/refactor/rename-message-type
* Rename package `openscenario_msgs` to `traffic_simulator_msgs`
* Update `UserDefinedValueCondition` to reverive name of path-like pattern
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/behavior_plugin
* Contributors: MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, yamacir-kit

0.5.5 (2021-10-13)
------------------
* Merge pull request `#561 <https://github.com/tier4/scenario_simulator_v2/issues/561>`_ from tier4/fix/interpreter/reach-position-condition
* Cleanup
* Update `ReachPositionCondition` to not to use `API::reachPosition`
* Contributors: Tatsuya Yamasaki, yamacir-kit

0.5.4 (2021-10-13)
------------------
* Merge pull request `#557 <https://github.com/tier4/scenario_simulator_v2/issues/557>`_ from tier4/revert/pr_544
* Revert "Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status"
* Merge pull request `#554 <https://github.com/tier4/scenario_simulator_v2/issues/554>`_ from tier4/feature/autoware/upper-bound-velocity
* Merge remote-tracking branch 'origin/master' into feature/autoware/upper-bound-velocity
* Fix Autoware's default upper bound speed to double max from zero
* Update syntax `ObjectController` to support `assign` member function
* Update syntax `AssignControllerAction` to be optional
* Update syntax `OverrideControllerValueAction` to be optional
* Fix `setVehicleVelocity` to work in `Autoware::update`
* Add new member function `setUpperBoundSpeed`
* Rename member function `isEgo` to `isUserDefinedController`
* Revert changes for `ObjectController`
* Add member function `Controller::assign` for `AssignControllerAction`
* Remove type alias `DefaultController`
* Update Property/Properties operator []
* Lipsticks
* Contributors: MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.3 (2021-10-07)
------------------

0.5.2 (2021-10-06)
------------------
* Merge pull request `#544 <https://github.com/tier4/scenario_simulator_v2/issues/544>`_ from tier4/feature/remove_none_status
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/remove_none_status
* Merge pull request `#532 <https://github.com/tier4/scenario_simulator_v2/issues/532>`_ from tier4/refactor/interpreter/speedup-build
* Move some classes constructors into .cpp (11)
* Move some classes constructors into .cpp (10)
* Move some classes constructors into .cpp (9)
* Move some classes constructors into .cpp (8)
* Move some classes constructors into .cpp (7)
* Move some classes constructors into .cpp (6)
* Move some classes constructors into .cpp (5)
* remove boost::none from getStandstillDuration function
* Move some classes constructors into .cpp (4)
* Move some classes constructors into .cpp (3)
* Move some classes constructors into .cpp (2)
* Move some classes constructors into .cpp
* Move class `Scope` member implementation into .cpp
* Move class `WorldPosition` member implementation into .cpp
* Move class `Waypoint` member implementation into .cpp
* Move class `Vehicle` member implementation into .cpp
* Move class `UserDefinedValueCondition` member implementation into .cpp
* Move class `UnsignedShort` member implementation into .cpp
* Move class `UnsignedInteger` member implementation into .cpp
* Move class `TriggeringEntitiesRule` member implementation into .cpp
* Move class `TriggeringEntities` member implementation into .cpp
* Move class `Trigger` member implementation into .cpp
* Move class `TrafficSignals` member implementation into .cpp
* Move class `TrafficSignalStateAction` member implementation into .cpp
* Move class `TrafficSignalState` member implementation into .cpp
* Move class `Property` member implementation into .cpp
* Move class `Private` member implementation into .cpp
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Move class `StoryboardElementStateCondition` member implementation into .cpp
* Move class `Storyboard` member implementation into .cpp
* Move class `Story` member implementation into .cpp
* Move class `StandStillCondition` member implementation into .cpp
* Move class `SpeedCondition` member implementation into .cpp
* Move class `SpeedAction` member implementation into .cpp
* Move class `SimulationTimeCondition` member implementation into .cpp
* Move class `ScenarioObject` member implementation into .cpp
* Move class `ScenarioDefinition` member implementation into .cpp
* Move class `Route` member implementation into .cpp
* Move class `RoadNetwork` member implementation into .cpp
* Move class `RelativeWorldPosition` member implementation into .cpp
* Move class `RelativeTargetSpeed` member implementation into .cpp
* Move class `RelativeDistanceCondition` member implementation into .cpp
* Move class `ReachPositionCondition` member implementation into .cpp
* Move class `TeleportAction` member implementation into .cpp
* Move class `TimeHeadwayCondition` member implementation into .cpp
* Move class `TrafficSignalCondition` member implementation into .cpp
* Move class `TrafficSignalController` member implementation into .cpp
* Move class `TrafficSignalControllerAction` member implementation into .cpp
* Move class `TrafficSignalControllerCondition` member implementation into .cpp
* Move class `Position` member implementation into .cpp
* Move class `Phase` member implementation into .cpp
* Move class `Performance` member implementation into .cpp
* Move class `Pedestrian` member implementation into .cpp
* Move class `ParameterSetAction` member implementation into .cpp
* Move class `ParameterMultiplyByValueRule` member implementation into .cpp
* Move class `ParameterModifyAction` member implementation into .cpp
* Move class `ParameterCondition` member implementation into .cpp
* Move class `ParameterCondition` member implementation into .cpp
* Move class `ParameterAddValueRule` member implementation into .cpp
* Move class `Orientation` member implementation into .cpp
* Move class `OpenScenario` member implementation into .cpp
* Move class `ObjectController` member implementation into .cpp
* Move class `MiscObject` member implementation into .cpp
* Move class `ManeuverGroup` member implementation into .cpp
* Move class `Maneuver` member implementation into .cpp
* Move class `LanePosition` member implementation into .cpp
* Move class `LaneChangeAction` member implementation into .cpp
* Move class `Integer` member implementation into .cpp
* Move class `InitActions` member implementation into .cpp
* Move class `Init` member implementation into .cpp
* Move class `File` member implementation into .cpp
* Move class `Event` member implementation into .cpp
* Move class `Double` member implementation into .cpp
* Move class `Dimensions` member implementation into .cpp
* Move class `DeleteEntityAction` member implementation into .cpp
* Move class `CustomCommandAction` member implementation into .cpp
* Merge remote-tracking branch 'origin/master' into refactor/interpreter/speedup-build
* Move class `ControllerAction` member implementation into .cpp
* Move class `Controller` member implementation into .cpp
* Move class `ConditionGroup` member implementation into .cpp
* Move class `Condition` member implementation into .cpp
* Move class `Command` member implementation into .cpp
* Move class `Color` member implementation into .cpp
* Move class `CollisionCondition` member implementation into .cpp
* Move class `Center` member implementation into .cpp
* Move class `BoundingBox` member implementation into .cpp
* Move class `Boolean` member implementation into .cpp
* Move class `Axles` member implementation into .cpp
* Move class `Axle` member implementation into .cpp
* Move class `AssignRouteAction` member implementation into .cpp
* Move class `AssignControllerAction` member implementation into .cpp
* Move class `Arrow` member implementation into .cpp
* Move class `AddEntityAction` member implementation into .cpp
* Move class `Action` member implementation into .cpp
* Move class `Act` member implementation into .cpp
* Move class `AcquirePositionAction` member implementation into .cpp
* Move class `AccelerationCondition` member implementation into .cpp
* Move class `AbsoluteTargetSpeed` member implementation into .cpp
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.5.1 (2021-09-30)
------------------
* Merge pull request `#533 <https://github.com/tier4/scenario_simulator_v2/issues/533>`_ from tier4/feature/interpreter/distance-condition
* Lipsticks
* Support `DistanceCondition::distance<entity, *, false>`
* Support `DistanceCondition::distance<lane, longitudinal, false>`
* Simplify `DistanceCondition::distance` definition with macro
* Fix DistanceCondition.relativeDistanceType to be optinal element
* Update member function `distance` to recognize all parameters
* Add member function `DistanceCondition::distance`
* Update syntax `DistanceCondition` members to match OpenSCENARIO 1.1
* Merge branch 'master' into fix/clean_directory_behavior
* Merge branch 'master' into rename_AA_launch_package
* Merge pull request `#491 <https://github.com/tier4/scenario_simulator_v2/issues/491>`_ from tier4/feature/interpreter/fault-injection
* Update `FaultInjectionAction` topic name to `/simulation/events`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge pull request `#522 <https://github.com/tier4/scenario_simulator_v2/issues/522>`_ from tier4/fix/interpreter/add-entity-action
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge pull request `#524 <https://github.com/tier4/scenario_simulator_v2/issues/524>`_ from tier4/fix/get-jerk-limit-from-object-controller
* Merge remote-tracking branch 'origin/master' into fix/interpreter/add-entity-action
* fix snake_case to lowerCamelCase
* get jerk limits from ObjectController's property
* Remove deprecated headers
* Update `RelativeDistanceCondition` to ignore entity that has not yet been added
* Merge pull request `#511 <https://github.com/tier4/scenario_simulator_v2/issues/511>`_ from tier4/feature/metrics_get_jerk_from_autoware
* Remove `Pointer::start`
* EntityManager has a node as rclcpp::node_interfaces::NodeTopicInterface to erase its type
* Cleanup syntax `SpeedAction`
* Update `SpeedAction` to not to execute action in `start`
* Move `SpeedActionTarget`'s member functions into .cpp
* Update syntax `Action` to call Element's `run` on runningState
* Add free function `apply` for `Action`
* Update PrivateActions to support member function `run`
* Add free function `apply` for syntax `LongitudinalAction`
* Simplify `PrivateAction::endsImmediately` with `apply`
* Add free function `apply` for syntax `PrivateAction`
* Update `CollisionCondition` to ignore entity that has not yet been added
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge branch 'master' into feature/metrics_get_jerk_from_autoware
* Merge pull request `#519 <https://github.com/tier4/scenario_simulator_v2/issues/519>`_ from tier4/feature/interpreter/distance-condition
* Lipsticks
* Support all parameter pattern of `RelativeDistanceCondition`
* Remove deprecated (since OSC 1.1) enumeration `cartesianDistance`
* Lipsticks
* Add `RelativeDistanceCondition::distance` function template specializations
* Move implementation of member function `distance` into `relative_distance_condition.cpp`
* Update `RelativeDistanceCondition` to consider all given enumeration
* Add new syntax `CoordinateSystem`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/standstill_metric
* Reorder data members of `RelativeDistanceCondition`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/distance-condition
* Merge pull request `#506 <https://github.com/tier4/scenario_simulator_v2/issues/506>`_ from tier4/feature/interpreter/add-entity-action
* Lipsticks
* Update `AddEntityAction` to treat various Position types
* Lipsticks
* Move entity spawning operation into syntax `AddEntityAction` from `ScenarioObject`
* Update syntax `TeleportAction` to spawn entity that have not yet been spawned
* Add new member function `ScenarioObject::activateSensors`
* Move ObjectController assignment into functor `spawn_entity`
* Add new member function `makeOutOfRangeMetric`
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Merge remote-tracking branch 'origin/master' into feature/interpreter/add-entity-action
* Rename function `spawn` to `applyAddEntityAction`
* Rename member function `Scope::addElement` to `Scope::insert`
* Update syntax `Entities` to store ScenarioObjects into Scope
* Rename `despawn` to `applyDeleteEntityAction`
* fix config to subscribe closest_jerk
* Remove elements of `RoadNetwork` from struct `GlobalEnvironment`
* Cleanup member function `Interpreter::makeCurrentConfiguration`
* Add new struct `Scope::GlobalEnvironment`
* Cleanup syntax `EntityAction`
* Cleanup syntax `TeleportAction`
* Update syntax `TeleportAction` to use function `overload`
* Add new test scenario `CustomCommandAction.FaultInjectionAction.yaml`
* Cleanup test scenario `autoware-simple.yaml`
* Update CustomCommandAction type `FaultInjectionAction` to publish `SimulationEvents`
* Comment-out dependency to `autoware_simulation_msgs`
* Add package `autoware_simulation_msgs` to dependency
* Lipsticks
* Remove initialize statatement from if (C++17)
* Merge remote-tracking branch 'origin/master' into feature/interpreter/fault-injection
* Update `UserDefinedValueCondition` to be more faster
* Support new UserDefinedValueCondition `<ENTITY-NAME>.currentState`
* Support new member function `API::getCurrentAction`
* Add experimental CustomCommandAction type `FaultInjectionAction`
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.5.0 (2021-09-09)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_helper
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/dockerfile
* Merge pull request `#503 <https://github.com/tier4/scenario_simulator_v2/issues/503>`_ from tier4/feature/cleanup_code
* fix typo
* fix typo in rviz
* fix typo
* fix typo of split
* fix typo of SCENARIO
* change aori to rage
* use foo/bar/baz
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_test_traffic_light
* Merge pull request `#490 <https://github.com/tier4/scenario_simulator_v2/issues/490>`_ from tier4/fix/scenario-object-scope
* add a new scenario to check duplicated parameter
* fix bug
* delay ambiguity check until reference
* trivial fix
* ScenarioObject creates the scope
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_traffic_light
* Merge pull request `#482 <https://github.com/tier4/scenario_simulator_v2/issues/482>`_ from tier4/feature/scenario_test_runner/launch-autoware-option
* Merge branch 'master' into add-goalpose
* Update class `TransitionAssertion` to use ROS parameter `initialize_duration`
* Support new option `record:=<boolean>`
* Support new option `initialize_duration`
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/test_simulation_interface
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Rename option `architecture-type` to `architecture_type`
* Feature/request acuire position in world coordinate (`#439 <https://github.com/tier4/scenario_simulator_v2/issues/439>`_)
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' into add-goalpose
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into feature/context_panel
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Yutaro Matsuura, kyabe2718, yamacir-kit

0.4.5 (2021-08-30)
------------------
* Merge pull request `#437 <https://github.com/tier4/scenario_simulator_v2/issues/437>`_ from RobotecAI/issue/AJD-237-remove_autoware_compilation_flag
* Merge pull request `#472 <https://github.com/tier4/scenario_simulator_v2/issues/472>`_ from tier4/fix/interpreter/misc
* Disable variable-name duplication check
* review changes
* apply clang-format
* cleanup
* make Autoware switch based on autoware_type parameter
* remove unnecessary autoware_def include from openscenario_interpreter
* openscenario_interpreter uses ROS param instead of build flag to determine autoware
* Merge pull request `#444 <https://github.com/tier4/scenario_simulator_v2/issues/444>`_ from tier4/feature/interpreter/cleanup-error-messages
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Fix to set missing testsuites-name
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/math_test
* Move result file generation into member function `Interpreter::set<T>`
* Update `Interpreter::on_configure` to use `withExceptionHandler`
* Add new member function `Interpreter::makeCurrentConfiguration`
* Lipsticks
* Reorder Interpreter's member functions to be lexicographically
* Add member function `Interpreter::currentLocalFrameRate`
* Remove macro `CATCH` from class `Interpreter`
* Update function `guard` to receive exception handler
* Move functions (in namespace record) into new `header record.hpp`
* Move function 'record_start' and 'record_end' into namespace 'record'
* Add member function 'Interpreter::publishCurrentContext'
* Lipsticks
* Lipsticks
* Merge remote-tracking branch 'origin/master' into feature/interpreter/cleanup-error-messages
* Lipsticks
* Merge branch 'master' into AJD-238_scenario_validation
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, Wojciech Jaworski, danielm1405, yamacir-kit

0.4.4 (2021-08-20)
------------------
* Merge pull request `#451 <https://github.com/tier4/scenario_simulator_v2/issues/451>`_ from tier4/feature/out-of-range-metric
* add OutOfRangeMetric when vehicle is spawned
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/add_cpp_scenarios
* Merge branch 'master' into feature/acc-vel-out-of-range
* Contributors: MasayaKataoka, Tatsuya Yamasaki, kyabe2718

0.4.3 (2021-08-17)
------------------
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/suppress_warnings
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/follow_front_entity_behavior
* Merge pull request `#430 <https://github.com/tier4/scenario_simulator_v2/issues/430>`_ from tier4/feature/interpreter/error-message
* Merge remote-tracking branch 'origin/master' into feature/interpreter/error-message
* Merge pull request `#420 <https://github.com/tier4/scenario_simulator_v2/issues/420>`_ from tier4/namespace
* Fix openscenario_interpreter to set certain directory name to suitename
* fix ego_count
* Cleanup
* Update script 'result_checker'
* Remove deprecated data members from Interpreter
* Update junit result types to be stream insertable
* Update Interpreter to store current-result as variant of JUnit5 element
* Add some type aliases
* Add new header 'junit5.hpp'
* Add new struct 'Failure' for experimental JUnit library
* fix bugs of name resolution with anonymous scope and change all-in-one.yaml to require name resolution
* Add new struct 'Error' for experimental JUnit library
* Add experimental JUnit library
* remove explicit from copy/move constructor
* Rename struct 'ScopeImpl' to 'EnvironmentFrame'
* Lipsticks
* Lipsticks
* Remove type alias 'Scope::Actor'
* Fix typos
* Merge remote-tracking branch 'origin/master' into namespace
* Lipsticks
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/add_cpp_scenarios
* fix
* trivial fix
* fix
* fix traffic signals
* Remove unneeded definitions
* Merge branch 'master' into namespace
* Merge branch 'master' into namespace
* fix scope
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.4.2 (2021-07-30)
------------------

0.4.1 (2021-07-30)
------------------
* Merge pull request `#409 <https://github.com/tier4/scenario_simulator_v2/issues/409>`_ from tier4/feature/autoware/pose-with-covariance
* Remove unneeded definitions
* Merge remote-tracking branch 'origin/master' into feature/autoware/pose-with-covariance
* Contributors: Masaya Kataoka, yamacir-kit

0.4.0 (2021-07-27)
------------------
* Merge pull request `#407 <https://github.com/tier4/scenario_simulator_v2/issues/407>`_ from tier4/feature/galactic_support
* Lipsticks
* Add free function 'doller' emulates shell's '$(...)' expression
* add build flags for galactic/foxy to the openscenario_interpreter package
* apply reformat
* add ifdef flags for rosdistro
* Merge pull request `#402 <https://github.com/tier4/scenario_simulator_v2/issues/402>`_ from tier4/feature/interpreter/logic-file
* Lipsticks
* Add member function 'get*MapFile' to struct Configuration
* Update class Configuration to assert given map_path
* Merge remote-tracking branch 'origin/master' into feature/interpreter/logic-file
* Merge pull request `#401 <https://github.com/tier4/scenario_simulator_v2/issues/401>`_ from tier4/fix/typo
* Remove some duplicated API's data members
* fix typo described in https://github.com/tier4/scenario_simulator_v2/issues/398
* Add new struct 'Configuration' for class 'API'
* Merge pull request `#397 <https://github.com/tier4/scenario_simulator_v2/issues/397>`_ from tier4/fix/interpreter/acquire-position-action
* Rename some functions
* Rename function 'requestAcquirePosition' to 'applyAcquirePositionAction'
* Cleanup syntax ReachPositionCondition
* Rename procedure 'isReachedPosition' to 'evaluateReachPositionCondition'
* Merge remote-tracking branch 'origin/master' into fix/interpreter/acquire-position-action
* Update syntax AcquirePositionAction's accomplishments check
* Lipsticks
* Update syntax AcquirePositionAction to receive WorldPosition as destination
* Merge pull request `#390 <https://github.com/tier4/scenario_simulator_v2/issues/390>`_ from tier4/feature/interpreter/traffic-signal-controller-condition
* Fix syntax Event and ManeuverGroup to be able to restart elements
* Update TrafficLightManager to store TrafficLight objects directly
* Add type alias 'LaneletId' to syntax TrafficSignalCondition
* Lipsticks
* Add some member functions (for debug) to TrafficSignalController
* Fix StoryboardElement Action to ignore maximum_execution_count
* Update syntax 'TrafficSignalCondition'
* Update enumeration 'Color' to support conversion from traffic_simulator's Color
* Merge remote-tracking branch 'origin/master' into feature/interpreter/traffic-signal-controller-condition
* Update enumeration 'Arrow' to support conversion from traffic_simulator's Arrow
* Update enumeration 'Color'
* Update enumeration 'Arrow'
* Support syntax TrafficSignalControllerCondition
* Contributors: Masaya Kataoka, MasayaKataoka, Tatsuya Yamasaki, yamacir-kit

0.3.0 (2021-07-13)
------------------
* Merge pull request `#386 <https://github.com/tier4/scenario_simulator_v2/issues/386>`_ from tier4/feature/interpreter/misc-object
* Move Storyboard's simulation time check into StoryboardElement
* Fix Storyboard to check entities are ready if only before start simulation
* Add utility function 'overload'
* Update dependency 'nlohmann/json' to install via rosdep
* Support syntax MiscObject
* Update struct MiscObject
* Lipsticks
* Support syntax 'MiscObjectCategory'
* Merge pull request `#383 <https://github.com/tier4/scenario_simulator_v2/issues/383>`_ from tier4/feature/interpreter/test-scenario
* Support TrafficSignalCondition
* Merge remote-tracking branch 'origin/master' into feature/interpreter/test-scenario
* Merge pull request `#384 <https://github.com/tier4/scenario_simulator_v2/issues/384>`_ from tier4/feature/interpreter/assign-route-action-with-world-position
* Merge remote-tracking branch 'origin/master' into feature/interpreter/assign-route-action-with-world-position
* Merge pull request `#328 <https://github.com/tier4/scenario_simulator_v2/issues/328>`_ from RobotecAI/pjaroszek/map_and_planning
* Remove duplicated period
* Update WorldPosition to be convertible with 'openscenario_msgs::msg::LaneletPose'
* Rename function 'toMapPose' to 'toWorldPosition'
* Merge branch 'master' into pjaroszek/map_and_planning
* Merge pull request `#377 <https://github.com/tier4/scenario_simulator_v2/issues/377>`_ from tier4/traffic_signal_actions
* fix bug
* fix some function name and fix Arrow and Color trivially
* trivial fix
* add support for TrafficSignalController.referece
* adapt formatting
* remove like_action.hpp
* Merge branch 'master' into traffic_signal_actions
* build with AUTOWARE_AUTO flag defined instead of AUTOWARE_ARCHITECTURE_PROPOSAL
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/get_waypoints_error_message
* Merge pull request `#378 <https://github.com/tier4/scenario_simulator_v2/issues/378>`_ from tier4/feature/ego-entity/acuquire-position-action
* Add member function 'description' to syntax UserDefinedValueCondition
* Add struct 'UserDefinedValueCondition'
* ament_clang_format
* Merge branch 'master' into traffic_signal_actions
* fix segv
* add support for TrafficSingnalAction
* TrafficSignalControllerAction can directly modify the phase of TrafficSignalController
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, danielm1405, kyabe2718, yamacir-kit

0.2.0 (2021-06-24)
------------------
* Merge pull request `#370 <https://github.com/tier4/scenario_simulator_v2/issues/370>`_ from tier4/feature/interpreter/context-2
* Disable debug print
* Lipsticks
* Add member function 'description' to syntax StoryboardElementStateCondition
* Add member function 'description' to syntax ParameterCondition
* Add member function 'distance' to syntax RelativeDistanceCondition
* Add member function 'description' to syntax DistanceCondition
* Add member function 'description' to syntax ReachPositionCondition
* Add member function 'description' to syntax SpeedCondition
* Add member function 'description' to syntax StandStillCondition
* Add member function 'description' to syntax AccelerationCondition
* Add member function 'description' to syntax TimeHeadwayCondition
* Add new function 'print_to'
* Lipsticks
* Add member function 'description' to syntax CollisionCondition
* Add member function 'description' to syntax TriggeringEntities
* Add member function 'description' to syntax TriggeringEntitiesRule
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/send_ego_command
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/send_ego_command
* Contributors: Masaya Kataoka, yamacir-kit

0.1.1 (2021-06-21)
------------------
* Merge pull request `#364 <https://github.com/tier4/scenario_simulator_v2/issues/364>`_ from tier4/feature/change_timeout_to_warning
* change warnings
* Merge pull request `#362 <https://github.com/tier4/scenario_simulator_v2/issues/362>`_ from tier4/feature/interpreter/scope
* Convert almost all of class Scope to be base-class from data member
* Rename identifier 'callIt' and 'applyIt' to 'invoke'
* Merge pull request `#344 <https://github.com/tier4/scenario_simulator_v2/issues/344>`_ from tier4/feature/interpreter/context
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Rename context member 'Overview' to 'CurrentStates'
* Merge pull request `#320 <https://github.com/tier4/scenario_simulator_v2/issues/320>`_ from tier4/relative_target_speed
* Rename context element to 'currentEvaluation' from 'description'
* Rename context element 'currentEvaluation' to 'currentValue'
* Rename Scope::scope to Scope::localScope
* Rename member function 'state' to 'currentState'
* Update Interpreter to publish JSON formatted execution context
* Add new package 'openscenario_interpreter_msgs'
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Merge branch 'master' into relative_target_speed
* Merge pull request `#361 <https://github.com/tier4/scenario_simulator_v2/issues/361>`_ from tier4/execution_timer
* Update syntax Private to print each PrivateAction's typename
* Update syntax Private to be printable as JSON
* not use operator()
* Update syntax InitActions to be printable as JSON
* clang-format
* fix trivial
* add support for SpeedTargetValueType.factor and add these changes to the ReleaseNotes
* replace is_complete_immediately to endsImmediately
* Update syntax Init to be printable as JSON
* Update syntax Init to hold InitActions as data member
* Move states overview into operator<< of syntax OpenScenario
* remove unused include
* Move JSON-to-string conversion into on_activate from operator<<
* if evaluation() is not in time, the interpreter will throw an error
* Update member function SimulationTimeCondition::description
* Add new implicit member function 'description' to class Pointer<T>
* Update syntax TriggeringEntities
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Update syntax Condition to print it's typename
* Update syntax Condition to be printable as JSON
* Update syntax ConditionGroup to be printable as JSON
* Merge branch 'master' into relative_target_speed
* Update syntax Trigger to be printable as JSON
* Update syntax Action to print it's element typename
* Update syntax Action to be printable as JSON
* Update syntax Event to be printable as JSON
* Update syntax Maneuver to be printable as JSON
* Update syntax ManeuverGroup to be printable as JSON
* Update struct Scope
* Update syntax Act to hold scope as base-class
* Update syntax Act to be printable as JSON
* Update syntax Story to be printable as JSON
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* Update syntax Storyboard to print each Story
* Update syntax Storyboard to be printable as JSON
* Update OpenScenario::operator<<
* Merge branch 'feature/interpreter/context' of github.com:tier4/scenario_simulator_v2 into feature/interpreter/context
* Add operator<< for json vs syntax OpenScenario
* Update interpreter's CMakeLists to link nlohmann/json
* Merge remote-tracking branch 'origin/master' into feature/interpreter/context
* clang-format
* fix link error
* fix control reacheas end of non-void function and remove useless include
* Actions in Init must be completed immediately
* Merge branch 'master' into relative_target_speed
* Update syntax Storyboard to be printable
* Update syntax OpenScenario to be printable
* replace ImplementationFault with UNSUPPORTED_SETTING_DETECTED
* Merge branch 'master' into relative_target_speed
* ignore SemanticError while lane-changing
* Merge branch 'master' into relative_target_speed
* add support for RelativeActionTarget
* Merge branch 'master' into relative_target_speed
* Revert "add RelativeTargetSpeed support to interpreter"
* add RelativeTargetSpeed support to interpreter
* Contributors: Masaya Kataoka, Tatsuya Yamasaki, kyabe2718, yamacir-kit

0.1.0 (2021-06-16)
------------------
* Merge remote-tracking branch 'origin/master' into fix/traffic-simulator/simulation-model-2
* Merge pull request `#349 <https://github.com/tier4/scenario_simulator_v2/issues/349>`_ from tier4/fix/typos-in-openscenario-dir
* Fix typos in the openscenario directory
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#334 <https://github.com/tier4/scenario_simulator_v2/issues/334>`_ from tier4/fix/typos-in-docs-and-comments
* Merge branch 'master' into fix/typos-in-docs-and-comments
* Fix typos and grammars in docs and comments
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#333 <https://github.com/tier4/scenario_simulator_v2/issues/333>`_ from tier4/feature/interpreter/release-notes
* Lipsticks
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/speed_up_npc_logic
* Merge pull request `#331 <https://github.com/tier4/scenario_simulator_v2/issues/331>`_ from tier4/feature/interpreter/traffic-signals
* Update Scope to store references to TrafficSignalController
* Rename variable 'scenario' to 'pathname'
* Add struct FileHeader into struct OpenScenario
* Cleanup struct 'OpenScenario'
* Add new struct OpenScenarioCategory
* Move struct OpenScenario's stream ouput operator into .cpp
* Move struct ScenarioDefinition into new header
* Rename scenario 'Autoware.TrafficSignals' to 'TrafficSignals'
* Update test scenario
* Merge pull request `#325 <https://github.com/tier4/scenario_simulator_v2/issues/325>`_ from tier4/feature/interpreter/error-type-2
* Simplify macro 'UNSUPPORTED_ELEMENT_SPECIFIED'
* Add utility function 'demangle'
* Remove macro 'THROW_UNSUPPORTED_ERROR'
* Remove macro 'UNSUPPORTED'
* Remove sstream inclusion
* Remove cat.hpp
* Remove almost all of std::stringstream use
* Remove some of std::stringstream use
* Lipsticks
* Cleanup Interpreter::withExceptionHandler
* Replace error types in error.hpp with common::(.*)Error
* Remove struct ConnectionError
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into fix/image_font_size
* Merge pull request `#319 <https://github.com/tier4/scenario_simulator_v2/issues/319>`_ from tier4/feature/interpreter/error-type
* Remove struct ImplementationFault
* Remove ImplementationFault from Action/Conditions
* Remove ImplementationFault from struct SpeedAction
* Remove ImplementationFault from struct RelativeWorldPosition
* Remove ImplementationFault from class Pointer
* Remove ImplementationFault from struct Expression
* Remove ImplementationFault from struct StoryboardElementState
* Remove ImplementationFault from struct Command
* Remove ImplementationFault from struct ParameterType
* Remove ImplementationFault from TriggeringEntitiesRule
* Remove ImplementationFault from struct StoryboardElementType
* Remove ImplementationFault from struct DynamicsDimension
* Remove ImplementationFault from struct DynamicsShape
* Remove ImplementationFault from struct RouteStrategy
* Remove ImplementationFault from struct Rule
* Remove ImplementationFault from struct ConditionEdge
* Remove ImplementationFault from struct Priority
* Remove ImplementationFault from struct ReferenceContext
* Remove ImplementationFault from struct SpeedTargetValueType
* Remove ImplementationFault from struct RelativeDistanceType
* Remove ImplementationFault from struct PedestrianCategory
* Move stream output operator's implementation into .cpp
* Move stream output operator's implementation into .cpp
* Remove ImplementationFault from struct LaneChangeAction
* Move stream output operator's implementation into .cpp
* Move stream output operator's implementation into .cpp
* Remove ImplementationFault from struct WorldPosition
* Move stream output operator's implementation into .cpp
* Move stream output operator's implementation into .cpp
* Remove ImplementationFault from struct Position
* Remove ImplementationFault from struct VehicleCategory
* Deprecate struct 'ImplementationFault'
* Merge pull request `#317 <https://github.com/tier4/scenario_simulator_v2/issues/317>`_ from tier4/fix/interpreter/priority
* Lipsticks
* Update type 'Priority' to accept value 'parallel'
* Merge branch 'master' of github.com:tier4/scenario_simulator.auto into fix/hold_stream
* Merge pull request `#315 <https://github.com/tier4/scenario_simulator_v2/issues/315>`_ from tier4/feature/use_ros_clock
* enable use raw ros timestamp
* Lipsticks
* remove verbose option
* apply reformat
* Merge pull request `#313 <https://github.com/tier4/scenario_simulator_v2/issues/313>`_ from tier4/fix/publish_npc_detection_result_in_map_frame
* Merge branch 'fix/publish_npc_detection_result_in_map_frame' of github.com:tier4/scenario_simulator.auto into fix/publish_npc_detection_result_in_map_frame
* Disable auto-sink feature
* Merge pull request `#312 <https://github.com/tier4/scenario_simulator_v2/issues/312>`_ from tier4/fix/interpreter/acquire-position-action
* Cleanup AcquirePositionAction
* Merge pull request `#309 <https://github.com/tier4/scenario_simulator_v2/issues/309>`_ from tier4/fix/interpreter/deactivation
* Update interpreter to start simulation time from 0 if there is no Ego vehicle
* Rename package 'junit' to 'simple_junit'
* Remove namespace 'common'
* Update interpreter to write JUnit file on deactivation phase
* Update interpreter to store current error status
* Rename package 'junit_exporter' to 'junit'
* Rename class 'JunitExporter' to 'TestSuites'
* Move package 'junit_exporter' into directory 'common'
* Cleanup
* Reverse arguments order of JunitExporter::addTestCase
* Lipsticks
* Rename 'visibility.h' to 'visibility.hpp'
* Merge pull request `#306 <https://github.com/tier4/scenario_simulator_v2/issues/306>`_ from tier4/feature/use_common_exception
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/use_common_exception
* Merge pull request `#307 <https://github.com/tier4/scenario_simulator_v2/issues/307>`_ from tier4/feature/rosbag-record
* Remove some options from command 'ros2 bag record'
* Lipsticks
* Remove debug print
* Update interpreter to start 'ros2 bag record' on configure phase
* remove traffic_simulator::SimulationRuntimeError
* Merge pull request `#305 <https://github.com/tier4/scenario_simulator_v2/issues/305>`_ from tier4/refactor/scenario-test-runner
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into feature/synchronize_clock
* Add interactive messages
* Merge pull request `#303 <https://github.com/tier4/scenario_simulator_v2/issues/303>`_ from tier4/feature/common-exception-package
* Update Interpreter to receive new common exception types
* Update concealer to use common::AutowareError
* Merge pull request `#302 <https://github.com/tier4/scenario_simulator_v2/issues/302>`_ from tier4/feature/error-handling-2
* Remove an misimplemented error throwing
* Merge branch 'master' of https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Merge pull request `#297 <https://github.com/tier4/scenario_simulator_v2/issues/297>`_ from tier4/feature/error-handling
* Update Interpreter to destruct simulator on deactivation phase
* Merge https://github.com/tier4/scenario_simulator.auto into feature/publish_clock
* Replace some of std::cout with RCLCPP_INFO_STREAM
* Remove deprecated header 'utility/verbose.hpp'
* Lipsticks
* Update error display names
* Remove debug codes
* Remove debug codes
* Move Interpreter::report into .cpp
* Update interpreter to use RCLCPP_INFO_STREAM
* Merge remote-tracking branch 'origin/master' into feature/error-handling
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
