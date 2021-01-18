cmake_minimum_required(VERSION 3.5)
project(simulation_metric_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TraveledDistanceMetric.msg
  msg/MomentaryStopMetric.msg
  msg/DefaultParameters.msg
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
