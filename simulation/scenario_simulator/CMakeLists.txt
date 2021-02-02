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

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(scenario_simulator SHARED
  src/xmlrpc_method.cpp
  src/scenario_simulator_impl.cpp
  src/scenario_simulator.cpp
)

ament_auto_add_executable(scenario_simulator_node
  src/scenario_simulator_node.cpp
)
target_link_libraries(scenario_simulator_node
  scenario_simulator
)

install(TARGETS
  scenario_simulator_node
  DESTINATION lib/scenario_simulator
)

install(TARGETS scenario_simulator
EXPORT export_scenario_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
