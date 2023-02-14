cmake_minimum_required(VERSION 3.5)
project(traffic_simulator)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Add compile definition for switching to_yaml function's namespace. 
# (This function's namespace was changed in galactic => humble)
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(traffic_simulator_msgs REQUIRED)
find_package(Boost COMPONENTS filesystem)
find_package(lanelet2_matching REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(pluginlib REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(traffic_simulator SHARED
  src/api/api.cpp
  src/behavior/route_planner.cpp
  src/behavior/longitudinal_speed_planning.cpp
  src/color_utils/color_utils.cpp
  src/data_type/behavior.cpp
  src/data_type/lane_change.cpp
  src/data_type/speed_change.cpp
  src/entity/ego_entity.cpp
  src/entity/entity_base.cpp
  src/entity/entity_manager.cpp
  src/entity/misc_object_entity.cpp
  src/entity/pedestrian_entity.cpp
  src/entity/vehicle_entity.cpp
  src/hdmap_utils/hdmap_utils.cpp
  src/helper/helper.cpp
  src/job/job.cpp
  src/job/job_list.cpp
  src/metrics/metric_base.cpp
  src/metrics/metrics_manager.cpp
  src/metrics/momentary_stop_metric.cpp
  src/metrics/out_of_range_metric.cpp
  src/metrics/reaction_time_metric.cpp
  src/simulation_clock/simulation_clock.cpp
  src/traffic/traffic_controller.cpp
  src/traffic/traffic_sink.cpp
  src/traffic_lights/traffic_light.cpp
  src/traffic_lights/traffic_light_manager.cpp
  src/vehicle_model/sim_model_delay_steer_acc.cpp
  src/vehicle_model/sim_model_delay_steer_acc_geared.cpp
  src/vehicle_model/sim_model_delay_steer_vel.cpp
  src/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/vehicle_model/sim_model_interface.cpp
)

target_link_libraries(traffic_simulator
  tinyxml2
  pugixml
  pthread
  sodium
  zmq
  stdc++fs
  Boost::filesystem
  ${PROTOBUF_LIBRARY})

# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(traffic_simulator PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

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
