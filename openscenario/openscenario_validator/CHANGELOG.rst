^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openscenario_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.7 (2024-06-19)
------------------
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Merge branch 'master' into feature/improve-ros-parameter-handling
* Contributors: Masaya Kataoka, Mateusz Palczuk

2.1.6 (2024-06-18)
------------------

2.1.5 (2024-06-18)
------------------

2.1.4 (2024-06-14)
------------------
* Merge branch 'master' into fix/remove_quaternion_operation
* Merge branch 'master' into fix/remove_quaternion_operation
* Contributors: Masaya Kataoka

2.1.3 (2024-06-14)
------------------
* Merge branch 'master' into fix/issue1276
* Contributors: Masaya Kataoka

2.1.2 (2024-06-13)
------------------
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge branch 'master' into fix/interpreter/fault-injection-action
* Merge remote-tracking branch 'origin/master' into fix/interpreter/fault-injection-action
* Contributors: Tatsuya Yamasaki, yamacir-kit

2.1.1 (2024-06-11)
------------------
* Merge branch 'master' into fix/reorder
* Merge branch 'master' into fix/reorder
* Merge branch 'master' of https://github.com/tier4/scenario_simulator_v2 into fix/reorder
* Contributors: Kotaro Yoshimoto, hakuturu583

2.1.0 (2024-06-11)
------------------
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Merge branch 'master' into fix/RJD-955-fix-followtrajectoryaction-nan-time
* Contributors: DMoszynski, Tatsuya Yamasaki

2.0.5 (2024-06-11)
------------------
* merge / resolve confict
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/longitudinal_speed_planner
* Contributors: robomic

2.0.4 (2024-06-10)
------------------
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* Merge branch 'master' of github.com:tier4/scenario_simulator_v2 into feature/unit_tests/hdmap_utils
* Contributors: robomic

2.0.3 (2024-06-10)
------------------
* Merge branch 'master' into fix/remove_linear_algebra
* Contributors: Taiga

2.0.2 (2024-06-03)
------------------

2.0.1 (2024-05-30)
------------------
* Merge pull request `#1254 <https://github.com/tier4/scenario_simulator_v2/issues/1254>`_ from tier4/refactor/openscenario_validator
  refactor `openscenario validator` package
* refactor: validator.hpp
* Merge branch 'master' into refactor/openscenario_validator
* Merge branch 'master' into refactor/openscenario_validator
* refactor: use noNamespaceSchemaLocation for validation in openscenario_validator
* refactor: delete in-memory xml schema from openscenario_validator
* Contributors: Kotaro Yoshimoto

2.0.0 (2024-05-27)
------------------
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Merge branch 'master' into ref/RJD-1054-implement-distance-utils
* Contributors: DMoszynski, Masaya Kataoka, Tatsuya Yamasaki

1.18.0 (2024-05-24)
-------------------
* Merge branch 'master' into feature/traffic-source
* Merge branch 'master' into feature/traffic-source
* Contributors: Tatsuya Yamasaki

1.17.2 (2024-05-22)
-------------------

1.17.1 (2024-05-21)
-------------------

1.17.0 (2024-05-16)
-------------------
* Merge pull request `#1228 <https://github.com/tier4/scenario_simulator_v2/issues/1228>`_ from tier4/feature/openscenario_validator
  Add `openscenario_validator` package
* chore: update new package version
* refactor: delete unused function
* chore: update new package version
* refactor: rename variable and fix variable initialization
* refactor: load xsd from memory directly
* chore: update new package version
* refactor: manage XercesC lifecycle by XMLPlatformLifecycleHandler
* chore: update new package version
* chore: update new package version
* chore: update version of openscenario_validator
* fix: adjust export settings of openscenario_validator
* refactor: stop using nifty counter in OpenSCENARIOValidator
* fix: cmake configuration for openscenario_validator
* chore: update new package version
* chore: apply format
* refactor: rewrite xerces initialization
* refactor: rename openscenario_validator command name from validator into validate
* chore: use ${CMAKE_CURRENT_SOURCE_DIR} as root path for xsd file
  Co-authored-by: Tatsuya Yamasaki <httperror@404-notfound.jp>
* chore: follow package version
* chore: follow package version
* feat: add executable binary for openscenario_validator
* refactor: simplify CMakeLists.txt in openscenario_validator
* apply linter
* feat: update OpenSCENARIO version for openscenario_validator
* chore: update openscenario_validator version
* apply yamasaki-san's second patch
* apply yamasaki-san's patch
* Contributors: Kotaro Yoshimoto, Tatsuya Yamasaki
