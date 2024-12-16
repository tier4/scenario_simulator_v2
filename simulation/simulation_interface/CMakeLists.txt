cmake_minimum_required(VERSION 3.5)
project(simulation_interface)

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

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED thread)
find_package(rclcpp REQUIRED)
include(FindProtobuf REQUIRED)
ament_auto_find_build_dependencies()

set(PROTO_FILES
  "proto/autoware_control_msgs.proto"
  "proto/autoware_vehicle_msgs.proto"
  "proto/builtin_interfaces.proto"
  "proto/geometry_msgs.proto"
  "proto/rosgraph_msgs.proto"
  "proto/simulation_api_schema.proto"
  "proto/std_msgs.proto"
  "proto/traffic_simulator_msgs.proto"
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
include_directories(
  include
  ${CMAKE_BINARY_DIR}
)

ament_auto_add_library(simulation_interface SHARED
  src/zmq_multi_server.cpp
  src/zmq_multi_client.cpp
  src/conversions.cpp
  src/constants.cpp
  ${PROTO_SRCS}
)
target_link_libraries(simulation_interface
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

install(TARGETS simulation_interface
  EXPORT export_simulation_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  FILES ${PROTO_HDRS}
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversion test/test_conversions.cpp)
  target_link_libraries(test_conversion simulation_interface)
endif()

ament_auto_package()
