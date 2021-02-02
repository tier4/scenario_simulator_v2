cmake_minimum_required(VERSION 3.5)
project(openscenario_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoundingBox.msg
  msg/EntityType.msg
  msg/EntityStatus.msg
  msg/LaneletPose.msg
  msg/LaneletPoseAndStatus.msg
  msg/MapPoseAndStatus.msg
  msg/ActionStatus.msg
  msg/HermiteCurve.msg
  msg/CatmullRomSpline.msg
  msg/EntityTrajectory.msg
  msg/WaypointsArray.msg
  msg/EntityStatusWithTrajectory.msg
  msg/EntityStatusWithTrajectoryArray.msg
  msg/Obstacle.msg
  msg/DriverModel.msg
  msg/Performance.msg
  msg/Axle.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()