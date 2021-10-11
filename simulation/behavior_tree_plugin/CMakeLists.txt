cmake_minimum_required(VERSION 3.8)
project(behavior_tree_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(behavior_tree_plugin SHARED
  src/action_node.cpp
  src/behavior_tree_plugin.cpp
  src/pedestrian/behavior_tree.cpp
  src/pedestrian/follow_lane_action.cpp
  src/pedestrian/pedestrian_action_node.cpp
  src/pedestrian/walk_straight_action.cpp
  src/vehicle/behavior_tree.cpp
  src/vehicle/follow_lane_sequence/follow_front_entity_action.cpp
  src/vehicle/follow_lane_sequence/follow_lane_action.cpp
  src/vehicle/follow_lane_sequence/move_backward_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_traffic_light_action.cpp
  src/vehicle/follow_lane_sequence/yield_action.cpp
  src/vehicle/lane_change_action.cpp
  src/vehicle/vehicle_action_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
