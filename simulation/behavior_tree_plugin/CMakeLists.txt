cmake_minimum_required(VERSION 3.8)
project(behavior_tree_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

pluginlib_export_plugin_description_file(traffic_simulator plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(($ENV{ROS_DISTRO} STREQUAL "jazzy" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 2.5.3) OR
  ($ENV{ROS_DISTRO} STREQUAL "humble" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 1.3.11))
  # For ROS 2 Jazzy and Humble, we need to use the scoped header install directory.
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()
