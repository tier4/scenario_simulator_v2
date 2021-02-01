cmake_minimum_required(VERSION 3.5)
project(simulation_api)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(entity_manager SHARED
  src/behavior/action_node.cpp
  src/behavior/pedestrian/behavior_tree.cpp
  src/behavior/pedestrian/follow_lane_action.cpp
  src/behavior/pedestrian/pedestrian_action_node.cpp
  src/behavior/vehicle/behavior_tree.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_lane_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp
  src/behavior/vehicle/follow_lane_sequence/yield_action.cpp
  src/behavior/vehicle/lane_change_action.cpp
  src/behavior/vehicle/vehicle_action_node.cpp
  src/behavior/route_planner.cpp
  src/color_utils/color_utils.cpp
  src/entity/ego_entity.cpp
  src/entity/entity_base.cpp
  src/entity/entity_manager.cpp
  src/entity/pedestrian_entity.cpp
  src/entity/vehicle_entity.cpp
  src/hdmap_utils/hdmap_utils.cpp
  src/helper/helper.cpp
  src/math/catmull_rom_spline.cpp
  src/math/collision.cpp
  src/math/hermite_curve.cpp
  src/math/polynomial_solver.cpp
  src/vehicle_model/sim_model_delay_steer_acc.cpp
  src/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/vehicle_model/sim_model_interface.cpp
  src/metrics/metric_base.cpp
  src/metrics/metrics_manager.cpp
  src/metrics/traveled_distance_metric.cpp
  src/metrics/momentary_stop_metric.cpp
  src/metrics/reaction_time_metric.cpp
  src/traffic_lights/traffic_light.cpp
  src/traffic_lights/traffic_light_manager.cpp
)

target_link_libraries(entity_manager pugixml)

ament_auto_add_library(simulation_api SHARED src/api/api.cpp)
target_link_libraries(simulation_api entity_manager ${PROTOBUF_LIBRARY})

ament_auto_add_executable(scenario_runner_moc_node src/moc/scenario_runner_moc_node.cpp)

target_link_libraries(scenario_runner_moc_node simulation_api)

ament_auto_add_executable(catmull_rom_visualization src/moc/catmull_rom_visualization.cpp)

target_link_libraries(catmull_rom_visualization simulation_api)

install(
  DIRECTORY config launch resource test/catalog test/map
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_math test/src/test_math.cpp)
  target_link_libraries(test_math simulation_api)

  ament_add_gtest(test_hdmap_utils test/src/test_hdmap_utils.cpp)
  target_link_libraries(test_hdmap_utils simulation_api)
endif()

ament_auto_package()
