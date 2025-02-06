cmake_minimum_required(VERSION 3.5)
project(context_gamma_scenarios)

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

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(context_gamma_scenario_node SHARED
  src/context_gamma_scenario_node.cpp
)
target_link_libraries(context_gamma_scenario_node
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

# ament_auto_add_executable(traffic_simulation_context_gamma_demo
#   src/traffic_simulation_demo.cpp
# )
# target_link_libraries(traffic_simulation_context_gamma_demo context_gamma_scenario_node)

option(BUILD_CPP_MOCK_SCENARIOS "Building the C++ scenarios" OFF)
if(BUILD_CPP_MOCK_SCENARIOS)
  add_subdirectory(src/context_gamma_planner)

endif()

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})
install(
  FILES cmake/add_context_gamma_scenario_test.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include(cmake/add_context_gamma_scenario_test.cmake)
  # add_context_gamma_scenario_test(${PROJECT_NAME} "traffic_simulation_context_gamma_demo" "70")
endif()

ament_auto_package(CONFIG_EXTRAS "${PROJECT_NAME}_ament_cmake-extras.cmake")
