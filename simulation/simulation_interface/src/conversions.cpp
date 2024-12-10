// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simulation_interface
{
void toProto(const geometry_msgs::msg::Point & p, geometry_msgs::Point & proto)
{
  proto.set_x(p.x);
  proto.set_y(p.y);
  proto.set_z(p.z);
}

void toMsg(const geometry_msgs::Point & proto, geometry_msgs::msg::Point & p)
{
  p.x = proto.x();
  p.y = proto.y();
  p.z = proto.z();
}

void toProto(const geometry_msgs::msg::Quaternion & q, geometry_msgs::Quaternion & proto)
{
  proto.set_x(q.x);
  proto.set_y(q.y);
  proto.set_z(q.z);
  proto.set_w(q.w);
}

void toMsg(const geometry_msgs::Quaternion & proto, geometry_msgs::msg::Quaternion & q)
{
  q.x = proto.x();
  q.y = proto.y();
  q.z = proto.z();
  q.w = proto.w();
}

void toProto(const geometry_msgs::msg::Pose & p, geometry_msgs::Pose & proto)
{
  toProto(p.position, *proto.mutable_position());
  toProto(p.orientation, *proto.mutable_orientation());
}

void toMsg(const geometry_msgs::Pose & proto, geometry_msgs::msg::Pose & p)
{
  toMsg(proto.position(), p.position);
  toMsg(proto.orientation(), p.orientation);
}

void toProto(const geometry_msgs::msg::Vector3 & v, geometry_msgs::Vector3 & proto)
{
  proto.set_x(v.x);
  proto.set_y(v.y);
  proto.set_z(v.z);
}

void toMsg(const geometry_msgs::Vector3 & proto, geometry_msgs::msg::Vector3 & v)
{
  v.x = proto.x();
  v.y = proto.y();
  v.z = proto.z();
}

void toProto(const geometry_msgs::msg::Twist & t, geometry_msgs::Twist & proto)
{
  toProto(t.linear, *proto.mutable_linear());
  toProto(t.angular, *proto.mutable_angular());
}

void toMsg(const geometry_msgs::Twist & proto, geometry_msgs::msg::Twist & t)
{
  toMsg(proto.linear(), t.linear);
  toMsg(proto.angular(), t.angular);
}

void toProto(const geometry_msgs::msg::Accel & a, geometry_msgs::Accel & proto)
{
  toProto(a.linear, *proto.mutable_linear());
  toProto(a.angular, *proto.mutable_angular());
}

void toMsg(const geometry_msgs::Accel & proto, geometry_msgs::msg::Accel & a)
{
  toMsg(proto.linear(), a.linear);
  toMsg(proto.angular(), a.angular);
}

void toProto(
  const traffic_simulator_msgs::msg::BoundingBox & box, traffic_simulator_msgs::BoundingBox & proto)
{
  toProto(box.center, *proto.mutable_center());
  toProto(box.dimensions, *proto.mutable_dimensions());
}

void toMsg(
  const traffic_simulator_msgs::BoundingBox & proto, traffic_simulator_msgs::msg::BoundingBox & box)
{
  toMsg(proto.center(), box.center);
  toMsg(proto.dimensions(), box.dimensions);
}

void toProto(
  const traffic_simulator_msgs::msg::Performance & performance,
  traffic_simulator_msgs::Performance & proto)
{
  proto.set_max_acceleration(performance.max_acceleration);
  proto.set_max_acceleration_rate(performance.max_acceleration_rate);
  proto.set_max_deceleration(performance.max_deceleration);
  proto.set_max_deceleration_rate(performance.max_deceleration_rate);
  proto.set_max_speed(performance.max_speed);
}

void toMsg(
  const traffic_simulator_msgs::Performance & proto,
  traffic_simulator_msgs::msg::Performance & performance)
{
  performance.max_acceleration = proto.max_acceleration();
  performance.max_acceleration_rate = proto.max_acceleration_rate();
  performance.max_deceleration = proto.max_deceleration();
  performance.max_deceleration_rate = proto.max_deceleration_rate();
  performance.max_speed = proto.max_speed();
}

void toProto(const traffic_simulator_msgs::msg::Axle & axle, traffic_simulator_msgs::Axle & proto)
{
  proto.set_position_x(axle.position_x);
  proto.set_position_z(axle.position_z);
  proto.set_track_width(axle.track_width);
  proto.set_wheel_diameter(axle.wheel_diameter);
  proto.set_max_steering(axle.max_steering);
}

void toMsg(const traffic_simulator_msgs::Axle & proto, traffic_simulator_msgs::msg::Axle & axle)
{
  axle.position_x = proto.position_x();
  axle.position_z = proto.position_z();
  axle.track_width = proto.track_width();
  axle.wheel_diameter = proto.wheel_diameter();
  axle.max_steering = proto.max_steering();
}

void toProto(
  const traffic_simulator_msgs::msg::Axles & axles, traffic_simulator_msgs::Axles & proto)
{
  toProto(axles.front_axle, *proto.mutable_front_axle());
  toProto(axles.rear_axle, *proto.mutable_rear_axle());
}

void toMsg(const traffic_simulator_msgs::Axles & proto, traffic_simulator_msgs::msg::Axles & axles)
{
  toMsg(proto.front_axle(), axles.front_axle);
  toMsg(proto.rear_axle(), axles.rear_axle);
}

void toProto(
  const traffic_simulator_msgs::msg::VehicleParameters & p,
  traffic_simulator_msgs::VehicleParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  toProto(p.axles, *proto.mutable_axles());
  toProto(p.performance, *proto.mutable_performance());
  proto.set_name(p.name);
}

void toMsg(
  const traffic_simulator_msgs::VehicleParameters & proto,
  traffic_simulator_msgs::msg::VehicleParameters & p)
{
  toMsg(proto.axles(), p.axles);
  toMsg(proto.bounding_box(), p.bounding_box);
  toMsg(proto.performance(), p.performance);
  p.name = proto.name();
}

void toProto(
  const traffic_simulator_msgs::msg::PedestrianParameters & p,
  traffic_simulator_msgs::PedestrianParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  proto.set_name(p.name);
}

void toMsg(
  const traffic_simulator_msgs::PedestrianParameters & proto,
  traffic_simulator_msgs::msg::PedestrianParameters & p)
{
  p.name = proto.name();
  toMsg(proto.bounding_box(), p.bounding_box);
}

void toProto(
  const traffic_simulator_msgs::msg::MiscObjectParameters & p,
  traffic_simulator_msgs::MiscObjectParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  proto.set_name(p.name);
}

void toMsg(
  const traffic_simulator_msgs::MiscObjectParameters & proto,
  traffic_simulator_msgs::msg::MiscObjectParameters & p)
{
  p.name = proto.name();
  toMsg(proto.bounding_box(), p.bounding_box);
}

void toProto(
  const traffic_simulator_msgs::msg::ActionStatus & s, traffic_simulator_msgs::ActionStatus & proto)
{
  proto.set_current_action(s.current_action);
  toProto(s.twist, *proto.mutable_twist());
  toProto(s.accel, *proto.mutable_accel());
  proto.set_linear_jerk(s.linear_jerk);
}

void toMsg(
  const traffic_simulator_msgs::ActionStatus & proto, traffic_simulator_msgs::msg::ActionStatus & s)
{
  s.current_action = proto.current_action();
  toMsg(proto.twist(), s.twist);
  toMsg(proto.accel(), s.accel);
  s.linear_jerk = proto.linear_jerk();
}

void toProto(
  const traffic_simulator_msgs::msg::LaneletPose & pose,
  traffic_simulator_msgs::LaneletPose & proto)
{
  proto.set_lanelet_id(pose.lanelet_id);
  proto.set_s(pose.s);
  proto.set_offset(pose.offset);
  toProto(pose.rpy, *proto.mutable_rpy());
}

void toMsg(
  const traffic_simulator_msgs::LaneletPose & proto,
  traffic_simulator_msgs::msg::LaneletPose & pose)
{
  pose.lanelet_id = proto.lanelet_id();
  pose.s = proto.s();
  pose.offset = proto.offset();
  toMsg(proto.rpy(), pose.rpy);
}

void toProto(
  const traffic_simulator_msgs::msg::EntityType & type, traffic_simulator_msgs::EntityType & proto)
{
  switch (type.type) {
    case traffic_simulator_msgs::msg::EntityType::EGO:
      proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO);
      break;
    case traffic_simulator_msgs::msg::EntityType::VEHICLE:
      proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_VEHICLE);
      break;
    case traffic_simulator_msgs::msg::EntityType::PEDESTRIAN:
      proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_PEDESTRIAN);
      break;
    case traffic_simulator_msgs::msg::EntityType::MISC_OBJECT:
      proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_MISC_OBJECT);
      break;
    default:
      // LCOV_EXCL_START
      std::string message =
        "type of the Entity Type is invalid!\ntype is " + std::to_string(type.type);
      THROW_SIMULATION_ERROR(message);
      // LCOV_EXCL_STOP
      break;
  }
}

void toMsg(
  const traffic_simulator_msgs::EntityType & proto, traffic_simulator_msgs::msg::EntityType & type)
{
  switch (proto.type()) {
    case traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO:
      type.type = traffic_simulator_msgs::msg::EntityType::EGO;
      break;
    case traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_VEHICLE:
      type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
      break;
    case traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_PEDESTRIAN:
      type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
      break;
    case traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_MISC_OBJECT:
      type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
      break;
    default:
      // LCOV_EXCL_START
      std::string message = "type of the Entity Type is invalid!";
      THROW_SIMULATION_ERROR(message);
      // LCOV_EXCL_STOP
      break;
  }
}

void toProto(
  const traffic_simulator_msgs::msg::EntitySubtype & subtype,
  traffic_simulator_msgs::EntitySubtype & proto)
{
  switch (subtype.value) {
    case traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::CAR:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::TRUCK:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRUCK);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::BUS:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BUS);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::TRAILER:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRAILER);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_MOTORCYCLE);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::BICYCLE:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BICYCLE);
      break;
    case traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN:
      proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_PEDESTRIAN);
      break;
    default:
      // LCOV_EXCL_START
      std::string message =
        "subtype of the Entity Type is invalid!\nsubtype is " + std::to_string(subtype.value);
      THROW_SIMULATION_ERROR(message);
      // LCOV_EXCL_STOP
      break;
  }
}

void toMsg(
  const traffic_simulator_msgs::EntitySubtype & proto,
  traffic_simulator_msgs::msg::EntitySubtype & subtype)
{
  switch (proto.value()) {
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRUCK:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::TRUCK;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BUS:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::BUS;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRAILER:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::TRAILER;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_MOTORCYCLE:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BICYCLE:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::BICYCLE;
      break;
    case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_PEDESTRIAN:
      subtype.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
      break;
    default:
      // LCOV_EXCL_START
      std::string message = "type of the Entity subtype is invalid!";
      THROW_SIMULATION_ERROR(message);
      // LCOV_EXCL_STOP
      break;
  }
}

void toProto(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  traffic_simulator_msgs::EntityStatus & proto)
{
  toProto(status.type, *proto.mutable_type());
  toProto(status.subtype, *proto.mutable_subtype());
  proto.set_time(status.time);
  proto.set_name(status.name);
  toProto(status.bounding_box, *proto.mutable_bounding_box());
  toProto(status.action_status, *proto.mutable_action_status());
  toProto(status.pose, *proto.mutable_pose());
  toProto(status.lanelet_pose, *proto.mutable_lanelet_pose());
  proto.set_lanelet_pose_valid(status.lanelet_pose_valid);
  // proto.PrintDebugString();
}

void toMsg(
  const traffic_simulator_msgs::EntityStatus & proto,
  traffic_simulator_msgs::msg::EntityStatus & status)
{
  toMsg(proto.type(), status.type);
  toMsg(proto.subtype(), status.subtype);
  status.time = proto.time();
  status.name = proto.name();
  toMsg(proto.bounding_box(), status.bounding_box);
  toMsg(proto.action_status(), status.action_status);
  toMsg(proto.pose(), status.pose);
  toMsg(proto.lanelet_pose(), status.lanelet_pose);
  status.lanelet_pose_valid = proto.lanelet_pose_valid();
}

void toProto(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  simulation_api_schema::EntityStatus & proto)
{
  toProto(status.type, *proto.mutable_type());
  toProto(status.subtype, *proto.mutable_subtype());
  proto.set_time(status.time);
  proto.set_name(status.name);
  toProto(status.action_status, *proto.mutable_action_status());
  toProto(status.pose, *proto.mutable_pose());
}

void toMsg(
  const simulation_api_schema::EntityStatus & proto,
  traffic_simulator_msgs::msg::EntityStatus & status)
{
  toMsg(proto.type(), status.type);
  toMsg(proto.subtype(), status.subtype);
  status.time = proto.time();
  status.name = proto.name();
  toMsg(proto.action_status(), status.action_status);
  toMsg(proto.pose(), status.pose);
  status.bounding_box = traffic_simulator_msgs::msg::BoundingBox();
  status.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
  status.lanelet_pose_valid = false;
}

void toProto(
  const builtin_interfaces::msg::Duration & duration, builtin_interfaces::Duration & proto)
{
  proto.set_sec(duration.sec);
  proto.set_nanosec(duration.nanosec);
}

void toMsg(const builtin_interfaces::Duration & proto, builtin_interfaces::msg::Duration & duration)
{
  duration.sec = proto.sec();
  duration.nanosec = proto.nanosec();
}

void toProto(const builtin_interfaces::msg::Time & time, builtin_interfaces::Time & proto)
{
  proto.set_sec(time.sec);
  proto.set_nanosec(time.nanosec);
}

void toMsg(const builtin_interfaces::Time & proto, builtin_interfaces::msg::Time & time)
{
  time.sec = proto.sec();
  time.nanosec = proto.nanosec();
}

void toProto(const rosgraph_msgs::msg::Clock & clock, rosgraph_msgs::Clock & proto)
{
  toProto(clock.clock, *proto.mutable_clock());
}

void toMsg(const rosgraph_msgs::Clock & proto, rosgraph_msgs::msg::Clock & clock)
{
  toMsg(proto.clock(), clock.clock);
}

void toProto(const std_msgs::msg::Header & header, std_msgs::Header & proto)
{
  proto.set_frame_id(header.frame_id);
  toProto(header.stamp, *proto.mutable_stamp());
}

void toMsg(const std_msgs::Header & proto, std_msgs::msg::Header & header)
{
  header.frame_id = proto.frame_id();
  toMsg(proto.stamp(), header.stamp);
}

void toProto(
  const autoware_control_msgs::msg::Lateral & message, autoware_control_msgs::Lateral & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  proto.set_steering_tire_angle(message.steering_tire_angle);
  proto.set_steering_tire_rotation_rate(message.steering_tire_rotation_rate);
}

void toMsg(
  const autoware_control_msgs::Lateral & proto, autoware_control_msgs::msg::Lateral & message)
{
  toMsg(proto.stamp(), message.stamp);
  message.steering_tire_angle = proto.steering_tire_angle();
  message.steering_tire_rotation_rate = proto.steering_tire_rotation_rate();
}

void toProto(
  const autoware_control_msgs::msg::Longitudinal & message,
  autoware_control_msgs::Longitudinal & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  proto.set_velocity(message.velocity);
  proto.set_acceleration(message.acceleration);
  proto.set_jerk(message.jerk);
}

void toMsg(
  const autoware_control_msgs::Longitudinal & proto,
  autoware_control_msgs::msg::Longitudinal & message)
{
  toMsg(proto.stamp(), message.stamp);
  message.velocity = proto.velocity();
  message.acceleration = proto.acceleration();
  message.jerk = proto.jerk();
}

void toProto(
  const autoware_control_msgs::msg::Control & message, autoware_control_msgs::Control & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  toProto(message.lateral, *proto.mutable_lateral());
  toProto(message.longitudinal, *proto.mutable_longitudinal());
}

void toMsg(
  const autoware_control_msgs::Control & proto, autoware_control_msgs::msg::Control & message)
{
  toMsg(proto.stamp(), message.stamp);
  toMsg(proto.lateral(), message.lateral);
  toMsg(proto.longitudinal(), message.longitudinal);
}

auto toProto(
  const autoware_vehicle_msgs::msg::GearCommand & message,
  autoware_vehicle_msgs::GearCommand & proto) -> void
{
  toProto(message.stamp, *proto.mutable_stamp());

#define CASE(NAME)                                                         \
  case autoware_vehicle_msgs::msg::GearCommand::NAME:                      \
    proto.set_command(autoware_vehicle_msgs::GearCommand_Constants::NAME); \
    break

  switch (message.command) {
    CASE(NONE);
    CASE(NEUTRAL);
    CASE(DRIVE);
    CASE(DRIVE_2);
    CASE(DRIVE_3);
    CASE(DRIVE_4);
    CASE(DRIVE_5);
    CASE(DRIVE_6);
    CASE(DRIVE_7);
    CASE(DRIVE_8);
    CASE(DRIVE_9);
    CASE(DRIVE_10);
    CASE(DRIVE_11);
    CASE(DRIVE_12);
    CASE(DRIVE_13);
    CASE(DRIVE_14);
    CASE(DRIVE_15);
    CASE(DRIVE_16);
    CASE(DRIVE_17);
    CASE(DRIVE_18);
    CASE(REVERSE);
    CASE(REVERSE_2);
    CASE(PARK);
    CASE(LOW);
    CASE(LOW_2);
  }
}

auto toMsg(
  const autoware_vehicle_msgs::GearCommand & proto,
  autoware_vehicle_msgs::msg::GearCommand & message) -> void
{
  toMsg(proto.stamp(), message.stamp);
  message.command = proto.command();
}

auto toProto(
  const std::tuple<autoware_control_msgs::msg::Control, autoware_vehicle_msgs::msg::GearCommand> &
    message,
  traffic_simulator_msgs::VehicleCommand & proto) -> void
{
  toProto(std::get<0>(message), *proto.mutable_control());
  toProto(std::get<1>(message), *proto.mutable_gear_command());
}

auto toProtobufMessage(const traffic_simulator_msgs::msg::Vertex & message)
  -> traffic_simulator_msgs::Vertex
{
  auto proto = traffic_simulator_msgs::Vertex();
  proto.set_time(message.time);
  toProto(message.position, *proto.mutable_position());
  return proto;
}

auto toROS2Message(const traffic_simulator_msgs::Vertex & proto)
  -> traffic_simulator_msgs::msg::Vertex
{
  auto message = traffic_simulator_msgs::msg::Vertex();
  message.time = proto.time();
  toMsg(proto.position(), message.position);
  return message;
}

auto toProtobufMessage(const traffic_simulator_msgs::msg::Polyline & message)
  -> traffic_simulator_msgs::Polyline
{
  auto proto = traffic_simulator_msgs::Polyline();
  for (const auto & vertex : message.vertices) {
    *proto.add_vertices() = toProtobufMessage(vertex);
  }
  return proto;
}

auto toROS2Message(const traffic_simulator_msgs::Polyline & proto)
  -> traffic_simulator_msgs::msg::Polyline
{
  auto message = traffic_simulator_msgs::msg::Polyline();
  for (const auto & vertex : proto.vertices()) {
    message.vertices.push_back(toROS2Message(vertex));
  }
  return message;
}

auto toProtobufMessage(const traffic_simulator_msgs::msg::PolylineTrajectory & message)
  -> traffic_simulator_msgs::PolylineTrajectory
{
  auto proto = traffic_simulator_msgs::PolylineTrajectory();
  proto.set_initial_distance_offset(message.initial_distance_offset);
  proto.set_dynamic_constraints_ignorable(message.dynamic_constraints_ignorable);
  proto.set_base_time(message.base_time);
  proto.set_closed(message.closed);
  *proto.mutable_shape() = toProtobufMessage(message.shape);
  return proto;
}

auto toROS2Message(const traffic_simulator_msgs::PolylineTrajectory & proto)
  -> traffic_simulator_msgs::msg::PolylineTrajectory
{
  auto message = traffic_simulator_msgs::msg::PolylineTrajectory();
  message.initial_distance_offset = proto.initial_distance_offset();
  message.dynamic_constraints_ignorable = proto.dynamic_constraints_ignorable();
  message.base_time = proto.base_time();
  message.closed = proto.closed();
  message.shape = toROS2Message(proto.shape());
  return message;
}
}  // namespace simulation_interface
