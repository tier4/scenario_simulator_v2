^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package context_gamma_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

18.0.1 (2025-10-16)
-------------------
* Merge branch 'master' into refactor/lidar_sensor
* Contributors: Kotaro Yoshimoto

18.0.0 (2025-10-15)
-------------------
* Merge branch 'master' into feature/small_fix_new_front_entity_detection_logic
* Contributors: Taiga

17.9.1 (2025-10-14)
-------------------
* Merge branch 'master' into fix/RJD-1921-fix-steering-when-autoware-is-overwritten
* Contributors: Kotaro Yoshimoto

17.9.0 (2025-10-14)
-------------------
* Merge pull request `#1651 <https://github.com/tier4/scenario_simulator_v2/issues/1651>`_ from tier4/feature/ra
  Feature/ra
* bump version
* bump version
* Rename linear program functions for improved clarity and consistency
* Refactor applyConstraintOnLine and optimizeVelocityWithConstraints functions for improved clarity and parameter naming consistency
* Add commnet
* Fix sign error in ego and other pose position updates in PedestrianPlugin::update
* Refactor velocity variable names in PedestrianPlugin::update for improved clarity
* Refactor applyConstraintOnLine and optimizeVelocityWithConstraints functions for improved clarity and parameter naming consistency
* Rename RVO_EPSILON to ORCA_EPSILON for consistency in solver.hpp and solver.cpp
* upgrade version
* Enhance comments in FollowLanePlanner for clarity on waypoint calculations
* Move goal_threshold to protected section in GoalPlannerBase
* Remove appendGoalPoses and appendGoalPoints methods from GoalPlannerBase
* Refactor setMaxSpeed method to remove reference from parameter
* feat: refactor CMakeLists.txt to simplify library addition
* feat: refactor ORCA implementation by removing math_utils and updating includes
* Merge branch 'feature/ra' of github.com:tier4/scenario_simulator_v2 into feature/ra
* feat: add ORCA algorithm implementation and refactor file structure
* feat: add ROS message converter for geometry types
* fix build error
* refactor: rename functions to lowerCamelCase
* refactor: rename functions to lowerCamelCase
* fix spell miss
* Add cross_2d function and update references in solver and orca implementations
* Refactor ellipse_radius function into a lambda for better readability and maintainability
* Add the "explicit" keyword to this constructor.
* Make getBlackBoardValues virtual in ActionNodeBase and add override in subclass
* Explicitly capture required local variables in lambda
* Use override specifier
* update version
* remove unuse code
* remove move
* Avoid unnecessary copy by using a const reference
* const reference in ResetRequestEvent constructor
* Merge nested if-statement into enclosing if condition
* Use in-class initializer for planner\_ instead of constructor initializer list.
* use structured binding
* Replace this use of "emplace" with "try_emplace".
* Make destructor virtual
* remove orverride
* Concatenate namespace with its nested namespace
* Remove unnecessary code
* Add package description
* Remove Unuse Include
* Consolidate scattered type conversions into a utility.
* Remove redundant current_action\_ declaration in derived class and use base class field
* Organized include files.
* Remove unnecessary comments and clean up code in action and planner classes
* Remove commented-out root_node\_ member from transition event classes
* Refactor include guards in transition event headers to use uppercase naming convention
* Update copyright year in multiple source files to 2015
* Update package version to 16.7.6 in package.xml
* Ignore entities marked with __CONTEXT_GAMMA_IGNORE_\_ in PedestrianPlugin update method
* Fix header guards and ensure proper formatting in multiple header and source files
* Fix header guard typos in follow_polyline_trajectory_action and follow_polyline_trajectory_planner_base headers
* Update velocity optimization to use planning speed in PedestrianPlugin
* Add ellipse_radius function to calculate ellipse radius based on bounding box and angles
* Remove collider utility and update references in context gamma planner
* Add collider and ORCA utilities for context gamma planner
* Add math utilities and solver for context gamma planner
* Remove unuse code
* Add context gamma base
* Contributors: Taiga, Taiga Takano
