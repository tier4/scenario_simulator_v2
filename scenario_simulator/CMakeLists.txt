cmake_minimum_required(VERSION 3.5)
project(scenario_simulator)

add_compile_options(-std=c++14)

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
find_package(xmlrpcpp REQUIRED)
find_package(simulation_controller REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(visualization_msgs REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  geometry_msgs
#  rclcpp
#  std_msgs
#  tf2
#  tf2_geometry_msgs
#  tf2_ros
#  simulation_controller
#  roslib
#  
#  
#)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(scenario_simulator SHARED
  src/xmlrpc_method.cpp
  src/scenario_simulator_impl.cpp
  src/scenario_simulator.cpp
)
ament_target_dependencies(scenario_simulator
  rclcpp
  xmlrpcpp
  simulation_controller
  quaternion_operation
  autoware_auto_msgs
  lanelet2_core
  lanelet2_routing
  lanelet2_extension
  visualization_msgs
)

#add_executable(scenario_simulator_node
#  src/scenario_simulator_node.cpp
#  
#  
#)
#target_link_libraries(scenario_simulator_node
#  ${catkin_LIBRARIES}
#  pugixml
#)

#add_executable(scenario_runner_moc_node
#  src/scenario_runner_moc_node.cpp
#)
#target_link_libraries(scenario_runner_moc_node
#  ${catkin_LIBRARIES}
#  pugixml
#)

#install(TARGETS
#  scenario_simulator_node scenario_runner_moc_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(
#  DIRECTORY
#    ${CMAKE_CURRENT_SOURCE_DIR}/launch
#    ${CMAKE_CURRENT_SOURCE_DIR}/config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_export_include_directories(include)
ament_package()