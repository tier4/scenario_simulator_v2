^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
