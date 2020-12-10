cmake_minimum_required(VERSION 3.5)
project(openscenario_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoundingBox.msg
  msg/EntityStatus.msg
  msg/EntityStatusArray.msg
  msg/LaneletPose.msg
  msg/LaneletPoseAndStatus.msg
  msg/MapPoseAndStatus.msg
  msg/ActionStatus.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()