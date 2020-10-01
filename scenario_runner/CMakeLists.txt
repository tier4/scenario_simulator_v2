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

find_package(geometry_msgs    REQUIRED)
find_package(lifecycle_msgs   REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs         REQUIRED)

# ------------------------------------------------------------------------------
#  libscenario_runner
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/object.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs)

target_link_libraries(${PROJECT_NAME} pugixml)

# ------------------------------------------------------------------------------
#  scenario_runner_node
# ------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# ------------------------------------------------------------------------------
#  make install
# ------------------------------------------------------------------------------

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config example include launch test
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(${PROJECT_NAME}_TEST_DEPENDENCIES
    ament_index_cpp
    geometry_msgs
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    std_msgs)

  ament_add_gtest(test_syntax test/test_syntax.cpp)
  if(TARGET test_syntax)
    ament_target_dependencies(test_syntax ${${PROJECT_NAME}_TEST_DEPENDENCIES})
    target_link_libraries(test_syntax ${PROJECT_NAME})
  endif()
endif()

ament_package()
