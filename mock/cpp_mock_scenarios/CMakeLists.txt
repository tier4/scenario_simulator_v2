cmake_minimum_required(VERSION 3.5)
project(cpp_mock_scenarios)

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

if(DEFINED SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO)
  if(${SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO} STREQUAL "awf/auto")
    add_compile_options(-DSCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO)
  endif()
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(cpp_scenario_node SHARED
  src/cpp_scenario_node.cpp
)
target_link_libraries(cpp_scenario_node
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmqpp
  zmq
)

ament_auto_add_executable(traffic_simulation_demo
  src/traffic_simulation_demo.cpp
)
target_link_libraries(traffic_simulation_demo cpp_scenario_node)

add_subdirectory(src/collision)
add_subdirectory(src/crosswalk)
add_subdirectory(src/follow_front_entity)
add_subdirectory(src/follow_lane)
add_subdirectory(src/lane_change)
add_subdirectory(src/measurement)
add_subdirectory(src/merge)
add_subdirectory(src/metrics)
add_subdirectory(src/move_backward)
add_subdirectory(src/pedestrian)
add_subdirectory(src/speed_planning)
add_subdirectory(src/traffic_light)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include(cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "traffic_simulation_demo" "70")
endif()

ament_auto_package(CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake")
