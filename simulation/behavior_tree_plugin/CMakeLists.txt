cmake_minimum_required(VERSION 3.8)
project(behavior_tree_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO)
  if(${SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO} STREQUAL "awf/auto")
    add_compile_options(-DSCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO)
  endif()
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_definitions(-DFOXY)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traffic_simulator REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)

add_library(behavior_tree_plugin SHARED
  src/action_node.cpp
  src/pedestrian/behavior_tree.cpp
  src/pedestrian/follow_lane_action.cpp
  src/pedestrian/pedestrian_action_node.cpp
  src/pedestrian/walk_straight_action.cpp
  src/transition_events/logging_event.cpp
  src/transition_events/reset_request_event.cpp
  src/transition_events/transition_event.cpp
  src/vehicle/behavior_tree.cpp
  src/vehicle/follow_lane_sequence/follow_front_entity_action.cpp
  src/vehicle/follow_lane_sequence/follow_lane_action.cpp
  src/vehicle/follow_lane_sequence/move_backward_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp
  src/vehicle/follow_lane_sequence/stop_at_traffic_light_action.cpp
  src/vehicle/follow_lane_sequence/yield_action.cpp
  src/vehicle/lane_change_action.cpp
  src/vehicle/vehicle_action_node.cpp
)

target_include_directories(
  behavior_tree_plugin
  PRIVATE
  include
)

ament_target_dependencies(behavior_tree_plugin
  rclcpp
  traffic_simulator
  behaviortree_cpp_v3
  pluginlib
)

pluginlib_export_plugin_description_file(traffic_simulator plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  behavior_tree_plugin
)
ament_export_dependencies(behavior_tree_plugin
  rclcpp
  traffic_simulator
  behaviortree_cpp_v3
  pluginlib
)

install(
  TARGETS behavior_tree_plugin
  DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

  ament_package()
