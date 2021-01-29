cmake_minimum_required(VERSION 3.5)
project(xmlrpc_protobuf)

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
find_package(xmlrpcpp REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_library(server
  src/server.cpp
)
ament_target_dependencies(server
  rclcpp
  xmlrpcpp
)
ament_export_targets(export_server HAS_LIBRARY_TARGET)
ament_export_libraries(server)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS server
EXPORT export_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_package()
