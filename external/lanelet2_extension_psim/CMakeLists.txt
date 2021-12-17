cmake_minimum_required(VERSION 3.5)
project(lanelet2_extension_psim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(lanelet2_extension_psim_lib SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/autoware_osm_parser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/autoware_traffic_light.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/detection_area.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/message_conversion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/mgrs_projector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/query.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/road_marking.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/road_marking.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/validation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/visualization.cpp
)

target_include_directories(lanelet2_extension_psim_lib PRIVATE
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS})

ament_auto_add_executable(sample_code
  src/sample_code.cpp
)

target_link_libraries(sample_code lanelet2_extension_psim_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_message_conversion test/src/test_message_conversion.cpp)
  if(TARGET test_message_conversion)
    ament_target_dependencies(test_message_conversion
      rclcpp
      lanelet2_core
      lanelet2_io
      lanelet2_maps
      lanelet2_projection
      lanelet2_routing
      lanelet2_traffic_rules
      lanelet2_validation
      autoware_auto_mapping_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_message_conversion lanelet2_extension_psim_lib)
  endif()

  ament_add_gtest(test_projector test/src/test_projector.cpp)
  if(TARGET test_projector)
    ament_target_dependencies(test_projector
      rclcpp
      lanelet2_core
      lanelet2_io
      lanelet2_maps
      lanelet2_projection
      lanelet2_routing
      lanelet2_traffic_rules
      lanelet2_validation
      autoware_auto_mapping_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_projector lanelet2_extension_psim_lib)
  endif()

  ament_add_gtest(test_query test/src/test_query.cpp)
  if(TARGET test_query)
    ament_target_dependencies(test_query
      rclcpp
      lanelet2_core
      lanelet2_io
      lanelet2_maps
      lanelet2_projection
      lanelet2_routing
      lanelet2_traffic_rules
      lanelet2_validation
      autoware_auto_mapping_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_query lanelet2_extension_psim_lib)
  endif()

  ament_add_gtest(test_regulatory_elements test/src/test_regulatory_elements.cpp)
  if(TARGET test_regulatory_elements)
    ament_target_dependencies(test_regulatory_elements
      rclcpp
      lanelet2_core
      lanelet2_io
      lanelet2_maps
      lanelet2_projection
      lanelet2_routing
      lanelet2_traffic_rules
      lanelet2_validation
      autoware_auto_mapping_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_regulatory_elements lanelet2_extension_psim_lib)
  endif()

  ament_add_gtest(test_utilities test/src/test_utilities.cpp)
  if(TARGET test_utilities)
    ament_target_dependencies(test_utilities
      rclcpp
      lanelet2_core
      lanelet2_io
      lanelet2_maps
      lanelet2_projection
      lanelet2_routing
      lanelet2_traffic_rules
      lanelet2_validation
      autoware_auto_mapping_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_utilities lanelet2_extension_psim_lib)
  endif()
endif()

ament_auto_package()
