cmake_minimum_required(VERSION 3.5)
project(traffic_simulator)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

if(DEFINED SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO)
  if(${SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO} STREQUAL "awf/auto")
    add_compile_options(-DSCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO)
  endif()
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(traffic_simulator_msgs REQUIRED)
find_package(Boost COMPONENTS filesystem)
find_package(lanelet2_matching REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(traffic_simulator SHARED
  src/api/api.cpp
  src/behavior/behavior_plugin_base.cpp
  src/behavior/route_planner.cpp
  src/behavior/target_speed_planner.cpp
  src/color_utils/color_utils.cpp
  src/entity/ego_entity.cpp
  src/entity/entity_base.cpp
  src/entity/entity_manager.cpp
  src/entity/misc_object_entity.cpp
  src/entity/pedestrian_entity.cpp
  src/entity/vehicle_entity.cpp
  src/hdmap_utils/hdmap_utils.cpp
  src/helper/helper.cpp
  src/math/bounding_box.cpp
  src/math/catmull_rom_spline.cpp
  src/math/collision.cpp
  src/math/distance.cpp
  src/math/hermite_curve.cpp
  src/math/linear_algebra.cpp
  src/math/polynomial_solver.cpp
  src/math/transform.cpp
  src/math/uuid.cpp
  src/metrics/collision_metric.cpp
  src/metrics/metric_base.cpp
  src/metrics/metrics_manager.cpp
  src/metrics/momentary_stop_metric.cpp
  src/metrics/out_of_range_metric.cpp
  src/metrics/reaction_time_metric.cpp
  src/metrics/standstill_metric.cpp
  src/metrics/traveled_distance_metric.cpp
  src/simulation_clock/simulation_clock.cpp
  src/traffic/traffic_controller.cpp
  src/traffic/traffic_sink.cpp
  src/traffic_lights/traffic_light.cpp
  src/traffic_lights/traffic_light_manager.cpp
  src/traffic_lights/traffic_light_state.cpp
  src/vehicle_model/sim_model_delay_steer_acc.cpp
  src/vehicle_model/sim_model_delay_steer_acc_geared.cpp
  src/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/vehicle_model/sim_model_interface.cpp
  src/vehicle_model/sim_model_time_delay.cpp
  src/vehicle_model/sim_model_util.cpp
)

target_link_libraries(traffic_simulator
  tinyxml2
  pugixml
  pthread
  sodium
  zmqpp
  zmq
  stdc++fs
  Boost::filesystem
  ${PROTOBUF_LIBRARY})

install(
  DIRECTORY config test/catalog test/map
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

ament_auto_package()
