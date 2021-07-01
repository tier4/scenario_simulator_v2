cmake_minimum_required(VERSION 3.5)
project(openscenario_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ActionStatus.msg
  msg/Axle.msg
  msg/Axles.msg
  msg/BoundingBox.msg
  msg/CatmullRomSpline.msg
  msg/DriverModel.msg
  msg/EntityStatus.msg
  msg/EntityStatusWithTrajectory.msg
  msg/EntityStatusWithTrajectoryArray.msg
  msg/EntityTrajectory.msg
  msg/EntityType.msg
  msg/HermiteCurve.msg
  msg/LaneletPose.msg
  msg/LaneletPoseAndStatus.msg
  msg/MapPoseAndStatus.msg
  msg/MiscObjectParameters.msg
  msg/Obstacle.msg
  msg/PedestrianParameters.msg
  msg/Performance.msg
  msg/Property.msg
  msg/VehicleParameters.msg
  msg/WaypointsArray.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
