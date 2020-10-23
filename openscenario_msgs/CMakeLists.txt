cmake_minimum_required(VERSION 3.5)
project(openscenario_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/EntityStatus.msg
  msg/EntityStatusArray.msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()