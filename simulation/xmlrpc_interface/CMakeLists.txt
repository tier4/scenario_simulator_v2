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
find_package(ament_cmake_auto REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
include(FindProtobuf REQUIRED)
ament_auto_find_build_dependencies()

set(PROTO_FILES
  "proto/geometry_msgs.proto"
  "proto/openscenario_msgs.proto"
  "proto/simulation_api_schema.proto"
  "proto/test.proto"
)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
add_library(proto_msgs SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(proto_msgs ${PROTOBUF_LIBRARY})

include_directories(
  include
  ${CMAKE_BINARY_DIR}
)

ament_auto_add_library(xmlrpc_interface
  src/conversions.cpp
)
target_link_libraries(xmlrpc_interface
  ${PROTOBUF_LIBRARY}
  Boost::thread
  proto_msgs
)

ament_auto_add_executable(example
  src/example.cpp
)
target_link_libraries(example
  xmlrpc_interface
  Boost::thread
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
  target_link_libraries(test_conversion xmlrpc_interface Boost::thread)
endif()

ament_package()
