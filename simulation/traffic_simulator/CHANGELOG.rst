^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
