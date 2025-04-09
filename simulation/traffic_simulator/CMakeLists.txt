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

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(traffic_simulator_msgs REQUIRED)
find_package(Boost COMPONENTS filesystem)
find_package(lanelet2_matching REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(pluginlib REQUIRED)
include(FindProtobuf REQUIRED)

# TODO(HansRobo): Remove this workaround once https://github.com/autowarefoundation/autoware_universe/issues/10410 is fixed
find_package(TinyXML2 CONFIG QUIET)
if(NOT TinyXML2_FOUND)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

ament_auto_find_build_dependencies()

ament_auto_add_library(traffic_simulator SHARED
  src/api/api.cpp
  src/behavior/follow_trajectory.cpp
  src/behavior/follow_waypoint_controller.cpp
  src/behavior/longitudinal_speed_planning.cpp
  src/behavior/route_planner.cpp
  src/color_utils/color_utils.cpp
  src/data_type/behavior.cpp
  src/data_type/entity_status.cpp
  src/data_type/lane_change.cpp
  src/data_type/lanelet_pose.cpp
  src/data_type/routing_graph_type.cpp
  src/data_type/speed_change.cpp
  src/entity/ego_entity.cpp
  src/entity/entity_base.cpp
  src/entity/entity_manager.cpp
  src/entity/misc_object_entity.cpp
  src/entity/pedestrian_entity.cpp
  src/entity/vehicle_entity.cpp
  src/hdmap_utils/hdmap_utils.cpp
  src/helper/helper.cpp
  src/helper/ostream_helpers.cpp
  src/job/job.cpp
  src/job/job_list.cpp
  src/lanelet_wrapper/lanelet_loader.cpp
  src/lanelet_wrapper/lanelet_map.cpp
  src/lanelet_wrapper/lanelet_wrapper.cpp
  src/lanelet_wrapper/pose.cpp
  src/lanelet_wrapper/route.cpp
  src/lanelet_wrapper/traffic_rules.cpp
  src/simulation_clock/simulation_clock.cpp
  src/traffic/traffic_controller.cpp
  src/traffic/traffic_sink.cpp
  src/traffic/traffic_source.cpp
  src/traffic_lights/configurable_rate_updater.cpp
  src/traffic_lights/traffic_light.cpp
  src/traffic_lights/traffic_light_marker_publisher.cpp
  src/traffic_lights/traffic_light_publisher.cpp
  src/traffic_lights/traffic_lights.cpp
  src/traffic_lights/traffic_lights_base.cpp
  src/utils/distance.cpp
  src/utils/lanelet_map.cpp
  src/utils/pose.cpp
)

ament_auto_add_library(visualization_component SHARED
  src/visualization/visualization_component.cpp
)

ament_auto_add_executable(visualization_node
  src/visualization/visualization_node.cpp
)
target_link_libraries(visualization_node visualization_component)

ament_export_libraries(visualization_component)

rclcpp_components_register_nodes(visualization_component "traffic_simulator::VisualizationComponent")

target_link_libraries(traffic_simulator
  tinyxml2
  pugixml
  pthread
  sodium
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
