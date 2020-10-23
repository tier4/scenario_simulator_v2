cmake_minimum_required(VERSION 3.5)
project(openscenario_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/EntityStatus.msg
  msg/EntityStatusArray.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()