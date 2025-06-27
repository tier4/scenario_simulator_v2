cmake_minimum_required(VERSION 3.16.3)

project(arithmetic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(($ENV{ROS_DISTRO} STREQUAL "jazzy" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 2.5.3) OR
  ($ENV{ROS_DISTRO} STREQUAL "humble" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 1.3.11))
  # For ROS 2 Jazzy and Humble, we need to use the scoped header install directory.
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()
