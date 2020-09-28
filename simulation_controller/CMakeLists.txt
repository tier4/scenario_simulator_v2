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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  roslib
#  spline_interpolation
#)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_controller
#  CATKIN_DEPENDS
#    roscpp
#    std_msgs
#    geometry_msgs
#    tf2_ros
#    tf2_geometry_msgs
#    roslib
#    visualization_msgs
#    lanelet2_extension
#    spline_interpolation
#    quaternion_operation
#    behaviortree_cpp_v3
#  DEPENDS system_lib
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(simulation_controller
  #src/entity/ego_entity.cpp
  #src/entity/vehicle_entity.cpp
  #src/entity/pedestrian_entity.cpp
  #src/entity/entity_base.cpp
  #src/entity/entity_manager.cpp
  #src/color_utils/color_utils.cpp
  #src/behavior/vehicle/behavior_tree.cpp
  #src/behavior/vehicle/follow_lane_action.cpp
  #src/behavior/vehicle/acquire_position_action.cpp
  #src/behavior/vehicle/lane_change_action.cpp
  #src/behavior/pedestrian/behavior_tree.cpp
  #src/behavior/pedestrian/follow_lane_action.cpp
  #src/behavior/pedestrian/acquire_position_action.cpp
  #src/behavior/pedestrian/lane_change_action.cpp
  #src/behavior/action_node.cpp
  #src/hdmap_utils/hdmap_utils.cpp
  #src/api/api.cpp
  #src/api/entity_api_impl.cpp
  #src/api/simulation_api_impl.cpp
  #src/math/hermite_curve.cpp
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

ament_export_include_directories(include)
ament_package()
