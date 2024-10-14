cmake_minimum_required(VERSION 3.5)
project(simple_sensor_simulator)

add_compile_options(-std=c++17)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

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

ament_auto_add_library(simple_sensor_simulator_component SHARED
  src/sensor_simulation/detection_sensor/detection_sensor.cpp
  src/sensor_simulation/lidar/lidar_sensor.cpp
  src/sensor_simulation/imu/imu_sensor.cpp
  src/sensor_simulation/lidar/raycaster.cpp
  src/sensor_simulation/occupancy_grid/occupancy_grid_sensor.cpp
  src/sensor_simulation/occupancy_grid/occupancy_grid_builder.cpp
  src/sensor_simulation/occupancy_grid/grid_traversal.cpp
  src/sensor_simulation/primitives/box.cpp
  src/sensor_simulation/primitives/primitive.cpp
  src/sensor_simulation/sensor_simulation.cpp
  src/simple_sensor_simulator.cpp
  src/vehicle_simulation/ego_entity_simulation.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_map_acc_geared.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_vel.cpp
  src/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/vehicle_simulation/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/vehicle_simulation/vehicle_model/sim_model_interface.cpp
)
target_link_libraries(simple_sensor_simulator_component
  pthread
  sodium
  zmq
)
ament_target_dependencies(simple_sensor_simulator_component
  simulation_interface
)
rclcpp_components_register_nodes(simple_sensor_simulator_component
  "simple_sensor_simulator::ScenarioSimulator")

ament_auto_add_executable(simple_sensor_simulator_node
  src/simple_sensor_simulator_node.cpp
)
target_link_libraries(simple_sensor_simulator_node
  simple_sensor_simulator_component
)

install(TARGETS
  simple_sensor_simulator_node
  DESTINATION lib/simple_sensor_simulator
)

install(TARGETS simple_sensor_simulator_component
EXPORT export_simple_sensor_simulator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()
ament_auto_package()
