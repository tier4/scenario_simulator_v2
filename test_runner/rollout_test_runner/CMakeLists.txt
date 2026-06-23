cmake_minimum_required(VERSION 3.5)
project(rollout_test_runner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/bag_topic_republisher.cpp
  src/ego_replay_source.cpp
  src/main.cpp
  src/rollout_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROTOBUF_LIBRARY}
  pthread
  sodium
  zmq
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
