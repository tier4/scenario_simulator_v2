cmake_minimum_required(VERSION 3.5)
project(traffic_simulator_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ActionStatus.msg
  msg/Axle.msg
  msg/Axles.msg
  msg/BehaviorParameter.msg
  msg/BoundingBox.msg
  msg/DynamicConstraints.msg
  msg/EntityStatus.msg
  msg/EntityStatusWithTrajectory.msg
  msg/EntityStatusWithTrajectoryArray.msg
  msg/EntitySubtype.msg
  msg/EntityType.msg
  msg/LaneletPose.msg
  msg/LaneletPoseAndStatus.msg
  msg/MapPoseAndStatus.msg
  msg/MiscObjectParameters.msg
  msg/Obstacle.msg
  msg/PedestrianParameters.msg
  msg/Performance.msg
  msg/Polyline.msg
  msg/PolylineTrajectory.msg
  msg/VehicleParameters.msg
  msg/Vertex.msg
  msg/WaypointsArray.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
