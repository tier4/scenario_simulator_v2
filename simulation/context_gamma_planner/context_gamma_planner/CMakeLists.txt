cmake_minimum_required(VERSION 3.8)
project(context_gamma_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)
#find_package (glog 0.6.0 REQUIRED)


ament_auto_find_build_dependencies()

ament_auto_add_library(context_gamma_planner SHARED
  src/behavior/action_node_base.cpp
  src/behavior/pedestrian/action_node.cpp
  src/behavior/pedestrian/follow_lane_action.cpp
  src/behavior/pedestrian/follow_polyline_trajectory_action.cpp
  src/behavior/vehicle/action_node.cpp
  src/behavior/vehicle/follow_lane_action.cpp
  src/behavior/vehicle/follow_polyline_trajectory_action.cpp
  src/behavior/vehicle/lane_change_action.cpp
  src/constraints/constraint_activator_base.cpp
  src/constraints/constraint_base.cpp
  src/constraints/pedestrian/constraint_activator.cpp
  src/constraints/road_edge_constraint.cpp
  src/constraints/stop_line_constraint.cpp
  src/constraints/traffic_light_constraint.cpp
  src/constraints/vehicle/constraint_activator.cpp
  src/pedestrian_plugin.cpp
  src/planner/goal_planner_base.cpp
  src/planner/follow_polyline_trajectory_planner_base.cpp
  src/planner/pedestrian/follow_lane_planner.cpp
  src/planner/pedestrian/follow_polyline_trajectory_planner.cpp
  src/planner/vehicle/follow_lane_planner.cpp
  src/planner/vehicle/follow_polyline_trajectory_planner.cpp
  src/planner/vehicle/lane_change_planner.cpp
  src/transition_events/logging_event.cpp
  src/transition_events/reset_request_event.cpp
  src/transition_events/transition_event.cpp
  src/vehicle_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  glog
)

pluginlib_export_plugin_description_file(traffic_simulator plugins.xml)

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(context_gamma_planner PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(
  DIRECTORY config launch #test/maps
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
