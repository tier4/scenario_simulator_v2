cmake_minimum_required(VERSION 3.5)
project(xmlrpc_interface)

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
find_package(ament_cmake REQUIRED)
find_package(xmlrpcpp)
include(FindProtobuf REQUIRED)

include_directories(
  include
  ${CMAKE_BINARY_DIR}
)

set(PROTO_FILES
  "proto/geometry_msgs.proto"
  "proto/openscenario_msgs.proto"
  "proto/simulation_api_schema.proto"
  "proto/test.proto"
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
add_library(xmlrpc_interface
  src/conversions.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)
ament_target_dependencies(xmlrpc_interface
  xmlrpcpp
)
target_link_libraries(xmlrpc_interface
  ${PROTOBUF_LIBRARY}
)
ament_export_targets(export_xmlrpc_interface HAS_LIBRARY_TARGET)
ament_export_libraries(xmlrpc_interface)

add_executable(example
  src/example.cpp
)
ament_target_dependencies(example
  xmlrpcpp
)
target_link_libraries(example
  xmlrpc_interface
)

install(TARGETS
  example
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS xmlrpc_interface
EXPORT export_xmlrpc_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversion test/test_conversions.cpp)
  target_link_libraries(test_conversion xmlrpc_interface)
endif()

ament_package()
