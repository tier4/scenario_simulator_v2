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

find_package(ament_cmake_auto REQUIRED)
find_package(openscenario_msgs REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(traffic_simulator SHARED
  src/api/api.cpp
  src/behavior/action_node.cpp
  src/behavior/pedestrian/behavior_tree.cpp
  src/behavior/pedestrian/follow_lane_action.cpp
  src/behavior/pedestrian/pedestrian_action_node.cpp
  src/behavior/pedestrian/walk_straight_action.cpp
  src/behavior/route_planner.cpp
  src/behavior/target_speed_planner.cpp
  src/behavior/vehicle/behavior_tree.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/follow_lane_action.cpp
  src/behavior/vehicle/follow_lane_sequence/move_backward_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp
  src/behavior/vehicle/follow_lane_sequence/stop_at_traffic_light_action.cpp
  src/behavior/vehicle/follow_lane_sequence/yield_action.cpp
  src/behavior/vehicle/lane_change_action.cpp
  src/behavior/vehicle/vehicle_action_node.cpp
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
  src/math/polynomial_solver.cpp
  src/math/transform.cpp
  src/math/uuid.cpp
  src/math/linear_algebra.cpp
  src/metrics/metric_base.cpp
  src/metrics/metrics_manager.cpp
  src/metrics/momentary_stop_metric.cpp
  src/metrics/reaction_time_metric.cpp
  src/metrics/traveled_distance_metric.cpp
  src/metrics/out_of_range_metric.cpp
  src/metrics/standstill_metric.cpp
  src/simulation_clock/simulation_clock.cpp
  src/traffic/traffic_controller.cpp
  src/traffic/traffic_sink.cpp
  src/traffic_lights/traffic_light.cpp
  src/traffic_lights/traffic_light_manager.cpp
  src/traffic_lights/traffic_light_state.cpp
  src/vehicle_model/sim_model_ideal.cpp
  src/vehicle_model/sim_model_interface.cpp
  src/vehicle_model/sim_model_time_delay.cpp
  src/vehicle_model/sim_model_util.cpp
)

target_link_libraries(traffic_simulator
  pugixml
  pthread
  sodium
  zmqpp
  zmq
  ${PROTOBUF_LIBRARY})

install(
  DIRECTORY resource test/catalog test/map
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

ament_auto_package()
