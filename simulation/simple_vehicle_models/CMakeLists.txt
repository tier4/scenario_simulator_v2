cmake_minimum_required(VERSION 3.5)
project(simple_vehicle_models)

add_compile_options(-std=c++17)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

ament_auto_add_library(simple_vehicle_models SHARED
  src/sim_model_interface.cpp
  src/sim_model_delay_steer_acc.cpp
  src/sim_model_delay_steer_acc_geared.cpp
  src/sim_model_delay_steer_acc_geared_for_diffusion_planner.cpp
  src/sim_model_delay_steer_acc_geared_wo_fall_guard.cpp
  src/sim_model_delay_steer_map_acc_geared.cpp
  src/sim_model_delay_steer_vel.cpp
  src/sim_model_ideal_steer_acc.cpp
  src/sim_model_ideal_steer_acc_geared.cpp
  src/sim_model_ideal_steer_vel.cpp
  src/sim_model_perfect_trajectory_tracker.cpp
)

# C ABI wrapper (VmModel) for driving the vehicle models from C / Python (ctypes) without ROS or
# ZeroMQ. Kept here so the models and their thin C surface share a single home (SSOT).
add_library(vehicle_model_wrapper SHARED
  src/vehicle_model_c_wrapper.cpp
)
target_include_directories(vehicle_model_wrapper PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)
target_compile_features(vehicle_model_wrapper PRIVATE cxx_std_17)
set_target_properties(vehicle_model_wrapper PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(vehicle_model_wrapper autoware_vehicle_msgs)
target_link_libraries(vehicle_model_wrapper simple_vehicle_models)
install(TARGETS vehicle_model_wrapper
  LIBRARY DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sim_model_perfect_trajectory_tracker
    test/test_sim_model_perfect_trajectory_tracker.cpp)
  target_link_libraries(test_sim_model_perfect_trajectory_tracker simple_vehicle_models)
endif()

ament_auto_package()
