cmake_minimum_required(VERSION 3.5)
project(simulation_api)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(spline_interpolation REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(openscenario_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(entity_manager SHARED
  src/hdmap_utils/hdmap_utils.cpp
  src/math/hermite_curve.cpp
  src/entity/entity_manager.cpp
  src/entity/entity_status.cpp
  src/entity/entity_base.cpp
  src/entity/vehicle_entity.cpp
  src/entity/ego_entity.cpp
  src/entity/pedestrian_entity.cpp
  src/behavior/action_node.cpp
  src/behavior/vehicle/behavior_tree.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_lane_action.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp
  src/behavior/vehicle/lane_change_action.cpp
  src/behavior/vehicle/acquire_position_action.cpp
  src/behavior/vehicle/vehicle_action_node.cpp
  src/behavior/pedestrian/behavior_tree.cpp
  src/behavior/pedestrian/follow_lane_action.cpp
  src/behavior/pedestrian/acquire_position_action.cpp
  src/behavior/pedestrian/pedestrian_action_node.cpp
  src/color_utils/color_utils.cpp
)
ament_target_dependencies(entity_manager
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  quaternion_operation
  autoware_auto_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_routing
  lanelet2_projection
  lanelet2_extension
  spline_interpolation
  ament_index_cpp
  behaviortree_cpp_v3
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp
  openscenario_msgs
)
target_link_libraries(entity_manager
  pugixml
)

add_library(api SHARED
  src/api/api.cpp
  src/api/entity_api_impl.cpp
)
ament_target_dependencies(api
  xmlrpcpp
  openscenario_msgs
)
target_link_libraries(api
  entity_manager
  pugixml
)

add_executable(scenario_runner_moc_node
  src/moc/scenario_runner_moc_node.cpp
)
ament_target_dependencies(scenario_runner_moc_node
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  visualization_msgs
  quaternion_operation
  autoware_auto_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_routing
  lanelet2_projection
  lanelet2_extension
  spline_interpolation
  ament_index_cpp
  behaviortree_cpp_v3
  xmlrpcpp
  openscenario_msgs
)
target_link_libraries(scenario_runner_moc_node
  api
)

install(TARGETS entity_manager api
  EXPORT export_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(entity_manager)
ament_export_interfaces(export_api HAS_LIBRARY_TARGET)
ament_export_libraries(api)


install(TARGETS
  scenario_runner_moc_node
  DESTINATION lib/simulation_api
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY resource launch config test/catalog test/map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_math test/src/test_math.cpp)
  target_link_libraries(test_math api)

  ament_add_gtest(test_hdmap_utils test/src/test_hdmap_utils.cpp)
  target_link_libraries(test_hdmap_utils api)
endif()

ament_export_include_directories(include)
ament_package()
