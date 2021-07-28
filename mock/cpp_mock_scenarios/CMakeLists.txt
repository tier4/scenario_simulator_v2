cmake_minimum_required(VERSION 3.5)
project(cpp_mock_scenarios)

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
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(view_kashiwanoha
  src/view_kashiwanoha.cpp
)
target_link_libraries(view_kashiwanoha
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmqpp
  zmq
)

ament_auto_add_executable(idiot_npc
  src/idiot_npc.cpp
)
target_link_libraries(idiot_npc
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmqpp
  zmq
)

install(
  DIRECTORY launch rviz test
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

