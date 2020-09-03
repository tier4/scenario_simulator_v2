cmake_minimum_required(VERSION 3.5)
project(lanelet2_extension)

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

add_compile_options(-std=c++14)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

add_definitions(${GeographicLib_DEFINITIONS})

add_library(lanelet2_extension_lib SHARED
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/detection_area.cpp
  lib/road_marking.cpp
  lib/mgrs_projector.cpp
  lib/message_conversion.cpp
  lib/query.cpp
  lib/road_marking.cpp
  lib/utilities.cpp
  lib/visualization.cpp
  lib/validation.cpp
)
ament_target_dependencies(lanelet2_extension_lib
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  autoware_lanelet2_msgs
  geometry_msgs
  visualization_msgs
)

ament_auto_add_executable(sample_code
  src/sample_code.cpp
)
target_link_libraries(sample_code lanelet2_extension_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS
  sample_code
  DESTINATION lib/lanelet2_extension
)
install(TARGETS lanelet2_extension_lib
  EXPORT export_lanelet2_extension_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
      autoware_lanelet2_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_message_conversion lanelet2_extension_lib)
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
      autoware_lanelet2_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_projector lanelet2_extension_lib)
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
      autoware_lanelet2_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_query lanelet2_extension_lib)
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
      autoware_lanelet2_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_regulatory_elements lanelet2_extension_lib)
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
      autoware_lanelet2_msgs
      geometry_msgs
      visualization_msgs
    )
    target_link_libraries(test_utilities lanelet2_extension_lib)
  endif()

endif()

ament_export_include_directories(include)
ament_export_interfaces(export_lanelet2_extension_lib HAS_LIBRARY_TARGET)
ament_package()