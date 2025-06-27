cmake_minimum_required(VERSION 3.5)
project(simple_junit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Boost COMPONENTS date_time filesystem)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/test_suites.cpp)

target_link_libraries(${PROJECT_NAME}
  Boost::date_time
  Boost::filesystem
  pugixml)

install(
  DIRECTORY test/expected
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_simple_junit test/src/test.cpp)
  target_link_libraries(test_simple_junit ${PROJECT_NAME})
endif()

if(($ENV{ROS_DISTRO} STREQUAL "jazzy" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 2.5.3) OR
  ($ENV{ROS_DISTRO} STREQUAL "humble" AND ${ament_cmake_auto_VERSION} VERSION_GREATER 1.3.11))
  # For ROS 2 Jazzy and Humble, we need to use the scoped header install directory.
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()
