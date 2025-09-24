cmake_minimum_required(VERSION 3.8)
project(trajectory_status_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(traffic_simulator REQUIRED)
find_package(traffic_simulator_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(trajectory_status_demo_node src/trajectory_status_demo_node.cpp)

ament_target_dependencies(trajectory_status_demo_node
  rclcpp
  std_msgs
  geometry_msgs
  traffic_simulator
  traffic_simulator_msgs
)

# Install targets
install(TARGETS
  trajectory_status_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()