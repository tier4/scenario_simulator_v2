cmake_minimum_required(VERSION 3.5)
project(scenario_runner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs     REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs          REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/object.cpp)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  rclcpp_components
  std_msgs)

target_link_libraries(${PROJECT_NAME}_node
  pugixml)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    config
    example
    include
    launch
  DESTINATION share/${PROJECT_NAME})

# install(
#   TARGETS
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
