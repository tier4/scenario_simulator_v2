cmake_minimum_required(VERSION 3.5)
project(concealer)

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

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware.cpp
  src/autoware_architecture_proposal.cpp
  src/autoware_auto.cpp
  src/autoware_universe.cpp
  src/execute.cpp
  src/task_queue.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
