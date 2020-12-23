cmake_minimum_required(VERSION 3.5)
project(awapi_awauto_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/awapi_awauto_adapter.cpp)

rclcpp_components_register_nodes(${PROJECT_NAME} "autoware_api::AutowareAutoAdapter")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/awapi_awauto_status_publisher.cpp
  src/awapi_lane_change_status_publisher.cpp
  src/awapi_obstacle_avoidance_status_publisher.cpp
  src/awapi_simulation_api_publisher.cpp
  src/awapi_vehicle_status_publisher.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
