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
find_package(agnocast_wrapper REQUIRED)

if(DEFINED ENV{ENABLE_AGNOCAST_SIMULATOR} AND "$ENV{ENABLE_AGNOCAST_SIMULATOR}" STREQUAL "1")
  message(WARNING "Building with agnocastlib dependency")
  find_package(agnocastlib REQUIRED)
else()
  message(WARNING "Building without agnocastlib dependency")
endif()

ament_auto_find_build_dependencies()

# --- PhysX backend for the taiga_x vehicle model (required) ------------------
# PhysX is provided by the physx_vendor package, which exports PHYSX_SDK_DIR /
# PHYSX_LIB_DIR pointing at the installed SDK (headers, release static libraries
# and the vehicle snippet sources).
find_package(physx_vendor REQUIRED)

set(_taiga_x_physx_libs
  PhysXVehicle2_static_64 PhysXExtensions_static_64 PhysXCooking_static_64
  PhysX_static_64 PhysXPvdSDK_static_64 PhysXCommon_static_64 PhysXFoundation_static_64)
if(NOT EXISTS "${PHYSX_SDK_DIR}/include/PxPhysicsAPI.h")
  message(FATAL_ERROR "taiga_x: PhysX headers not found at ${PHYSX_SDK_DIR}/include "
    "(build the physx_vendor package).")
endif()
foreach(_l IN LISTS _taiga_x_physx_libs)
  if(NOT EXISTS "${PHYSX_LIB_DIR}/lib${_l}.a")
    message(FATAL_ERROR "taiga_x: PhysX library not found: ${PHYSX_LIB_DIR}/lib${_l}.a")
  endif()
endforeach()
message(STATUS "taiga_x: using PhysX SDK at ${PHYSX_SDK_DIR}")

set(TAIGA_X_PHYSX_SOURCES
  src/vehicle_simulation/vehicle_model/sim_model_taiga_x.cpp
  ${PHYSX_SDK_DIR}/snippets/snippetvehiclecommon/base/Base.cpp
  ${PHYSX_SDK_DIR}/snippets/snippetvehiclecommon/directdrivetrain/DirectDrivetrain.cpp
  ${PHYSX_SDK_DIR}/snippets/snippetvehiclecommon/physxintegration/PhysXIntegration.cpp)
set(TAIGA_X_PHYSX_INCLUDES ${PHYSX_SDK_DIR}/include ${PHYSX_SDK_DIR}/snippets)
set(TAIGA_X_PHYSX_LINK_LIBS "")
foreach(_l IN LISTS _taiga_x_physx_libs)
  list(APPEND TAIGA_X_PHYSX_LINK_LIBS "${PHYSX_LIB_DIR}/lib${_l}.a")
endforeach()
list(APPEND TAIGA_X_PHYSX_LINK_LIBS pthread dl)

# Apply the PhysX backend (sources, includes, libs, defines) to a target.
function(taiga_x_apply_physx target)
  target_sources(${target} PRIVATE ${TAIGA_X_PHYSX_SOURCES})
  target_include_directories(${target} PRIVATE ${TAIGA_X_PHYSX_INCLUDES})
  target_link_libraries(${target} ${TAIGA_X_PHYSX_LINK_LIBS})
  target_compile_definitions(${target} PRIVATE PX_PHYSX_STATIC_LIB=1)
endfunction()

link_directories(${PCL_LIBRARY_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(simple_sensor_simulator_component SHARED
  src/sensor_simulation/detection_sensor/detection_sensor.cpp
  src/sensor_simulation/lidar/lidar_noise_model_v1.cpp
  src/sensor_simulation/lidar/lidar_sensor.cpp
  src/sensor_simulation/imu/imu_sensor.cpp
  src/sensor_simulation/lidar/raycaster.cpp
  src/sensor_simulation/occupancy_grid/grid_traversal.cpp
  src/sensor_simulation/occupancy_grid/occupancy_grid_builder.cpp
  src/sensor_simulation/occupancy_grid/occupancy_grid_sensor.cpp
  src/sensor_simulation/primitives/box.cpp
  src/sensor_simulation/primitives/primitive.cpp
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
  src/vehicle_simulation/vehicle_model/sim_model_perfect_trajectory_tracker.cpp
  src/vehicle_simulation/vehicle_model/sim_model_taiga_dyn.cpp
)
target_link_libraries(simple_sensor_simulator_component
  glog
  pthread
  sodium
  zmq
)
taiga_x_apply_physx(simple_sensor_simulator_component)
ament_target_dependencies(simple_sensor_simulator_component
  agnocast_wrapper
  simulation_interface
)

target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${agnocast_wrapper_INCLUDE_DIRS}
)

agnocast_wrapper_setup(${PROJECT_NAME}_component)
if(DEFINED ENV{ENABLE_AGNOCAST_SIMULATOR} AND "$ENV{ENABLE_AGNOCAST_SIMULATOR}" STREQUAL "1")
ament_target_dependencies(simple_sensor_simulator_component agnocastlib)
target_include_directories(${PROJECT_NAME}_component PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)
endif()

rclcpp_components_register_nodes(simple_sensor_simulator_component
  "simple_sensor_simulator::ScenarioSimulator")

ament_auto_add_executable(simple_sensor_simulator_node
  src/simple_sensor_simulator_node.cpp
)
target_link_libraries(simple_sensor_simulator_node
  simple_sensor_simulator_component
)

agnocast_wrapper_setup(${PROJECT_NAME}_node)

install(TARGETS
  simple_sensor_simulator_node
  DESTINATION lib/simple_sensor_simulator
)

install(TARGETS simple_sensor_simulator_component
EXPORT export_simple_sensor_simulator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(vehicle_model_wrapper SHARED
  src/vehicle_simulation/vehicle_model/vehicle_model_c_wrapper.cpp
  src/vehicle_simulation/vehicle_model/sim_model_interface.cpp
  src/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.cpp
  src/vehicle_simulation/vehicle_model/sim_model_taiga_dyn.cpp
)
target_include_directories(vehicle_model_wrapper PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)
target_compile_features(vehicle_model_wrapper PRIVATE cxx_std_17)
set_target_properties(vehicle_model_wrapper PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(vehicle_model_wrapper autoware_vehicle_msgs)
taiga_x_apply_physx(vehicle_model_wrapper)
install(TARGETS vehicle_model_wrapper
  LIBRARY DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()
ament_auto_package()
