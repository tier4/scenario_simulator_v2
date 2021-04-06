cmake_minimum_required(VERSION 3.5)
project(sensor_simulator)

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
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

link_directories(${PCL_LIBRARY_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(sensor_simulator_component SHARED
  src/sensor_simulator.cpp
  src/sensor_simulation/primitives/primitive.cpp
  src/sensor_simulation/primitives/box.cpp
  src/sensor_simulation/lidar/raycaster.cpp
  src/sensor_simulation/lidar/lidar_sensor.cpp
  src/sensor_simulation/sensor_simulation.cpp
  src/sensor_simulation/detection_sensor/detection_sensor.cpp
)
target_link_libraries(sensor_simulator_component
  embree3
  pthread
  sodium
  zmqpp
  zmq
)
ament_target_dependencies(sensor_simulator_component
  simulation_interface
)
rclcpp_components_register_nodes(sensor_simulator_component
  "sensor_simulator::ScenarioSimulator")

ament_auto_add_executable(sensor_simulator_node
  src/sensor_simulator_node.cpp
)
target_link_libraries(sensor_simulator_node
  sensor_simulator_component
)

install(TARGETS
  sensor_simulator_node
  DESTINATION lib/sensor_simulator
)

install(TARGETS sensor_simulator_component
EXPORT export_sensor_simulator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
