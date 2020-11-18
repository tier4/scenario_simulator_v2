cmake_minimum_required(VERSION 3.5)
project(awapi_accessor_example)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(awapi_accessor REQUIRED)
find_package(rclcpp         REQUIRED)

# ------------------------------------------------------------------------------
#  awapi_accessor_example_node
# ------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node awapi_accessor rclcpp)

# ------------------------------------------------------------------------------
#  make install
# ------------------------------------------------------------------------------

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

