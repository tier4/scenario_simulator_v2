cmake_minimum_required(VERSION 3.5)
project(simulation_controller)

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
find_package(rclcpp_components REQUIRED)
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

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(entity_manager SHARED
  src/hdmap_utils/hdmap_utils.cpp
  src/math/hermite_curve.cpp
  src/entity/entity_status.cpp
  src/entity/entity_base.cpp
  src/entity/vehicle_entity.cpp
  src/entity/ego_entity.cpp
  src/entity/pedestrian_entity.cpp
  src/entity/entity_manager.cpp
  src/behavior/action_node.cpp
  src/behavior/vehicle/behavior_tree.cpp
  src/behavior/vehicle/follow_lane_action.cpp
  src/behavior/vehicle/lane_change_action.cpp
  src/behavior/vehicle/acquire_position_action.cpp
  src/behavior/pedestrian/behavior_tree.cpp
  src/behavior/pedestrian/follow_lane_action.cpp
  src/behavior/pedestrian/acquire_position_action.cpp
  src/behavior/pedestrian/lane_change_action.cpp
  src/color_utils/color_utils.cpp
)
ament_target_dependencies(entity_manager
  rclcpp
  rclcpp_components
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
)
rclcpp_components_register_nodes(entity_manager
  "simulation_controller::entity::EntityManager")

add_library(api SHARED
  src/api/entity_api_impl.cpp
)
ament_target_dependencies(api
  xmlrpcpp
)
target_link_libraries(api
  entity_manager
)

install(TARGETS
  entity_manager api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(entity_manager)

#add_library(simulation_controller
  #src/api/api.cpp
  #src/api/entity_api_impl.cpp
  #src/api/simulation_api_impl.cpp
#)
#target_link_libraries(simulation_controller
#  pugixml
#  ${catkin_LIBRARIES}
#)

#install(TARGETS
#  simulation_controller
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(
#  DIRECTORY
#    ${CMAKE_CURRENT_SOURCE_DIR}/resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(
#  DIRECTORY
#    ${CMAKE_CURRENT_SOURCE_DIR}/test
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)

#  add_rostest_gtest(test-simulation_controller
#    test/test_simulation_controller.test
#    test/src/test_simulation_controller.cpp
#  )

#  target_link_libraries(test-simulation_controller
#    simulation_controller
#    ${catkin_LIBRARIES})
#endif ()

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
