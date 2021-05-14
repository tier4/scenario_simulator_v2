// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
  const openscenario_msgs::msg::BoundingBox & box, openscenario_msgs::BoundingBox & proto)
{
  toProto(box.center, *proto.mutable_center());
  toProto(box.dimensions, *proto.mutable_dimensions());
}

void toMsg(const openscenario_msgs::BoundingBox & proto, openscenario_msgs::msg::BoundingBox & box)
{
  toMsg(proto.center(), box.center);
  toMsg(proto.dimensions(), box.dimensions);
}

void toProto(
  const openscenario_msgs::msg::Performance & performance, openscenario_msgs::Performance & proto)
{
  proto.set_max_acceleration(performance.max_acceleration);
  proto.set_max_deceleration(performance.max_deceleration);
  proto.set_max_speed(performance.max_speed);
}

void toMsg(
  const openscenario_msgs::Performance & proto, openscenario_msgs::msg::Performance & performance)
{
  performance.max_acceleration = proto.max_acceleration();
  performance.max_deceleration = proto.max_deceleration();
  performance.max_speed = proto.max_speed();
}

void toProto(const openscenario_msgs::msg::Axle & axle, openscenario_msgs::Axle & proto)
{
  proto.set_position_x(axle.position_x);
  proto.set_position_z(axle.position_z);
  proto.set_track_width(axle.track_width);
  proto.set_wheel_diameter(axle.wheel_diameter);
  proto.set_max_steering(axle.max_steering);
}

void toMsg(const openscenario_msgs::Axle & proto, openscenario_msgs::msg::Axle & axle)
{
  axle.position_x = proto.position_x();
  axle.position_z = proto.position_z();
  axle.track_width = proto.track_width();
  axle.wheel_diameter = proto.wheel_diameter();
  axle.max_steering = proto.max_steering();
}

void toProto(const openscenario_msgs::msg::Axles & axles, openscenario_msgs::Axles & proto)
{
  toProto(axles.front_axle, *proto.mutable_front_axle());
  toProto(axles.rear_axle, *proto.mutable_rear_axle());
}

void toMsg(const openscenario_msgs::Axles & proto, openscenario_msgs::msg::Axles & axles)
{
  toMsg(proto.front_axle(), axles.front_axle);
  toMsg(proto.rear_axle(), axles.rear_axle);
}

/*
void toProto(
  const openscenario_msgs::msg::Property & p,
  openscenario_msgs::Property & proto)
{
  // proto.set_is_ego(p.is_ego);
}

void toMsg(
  const openscenario_msgs::Property & proto,
  openscenario_msgs::msg::Property & p)
{
  // p.is_ego = proto.is_ego();
}
*/

void toProto(
  const openscenario_msgs::msg::VehicleParameters & p, openscenario_msgs::VehicleParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  toProto(p.axles, *proto.mutable_axles());
  toProto(p.performance, *proto.mutable_performance());
  // toProto(p.property, *proto.mutable_property());
  proto.set_name(p.name);
  proto.set_vehicle_category(p.vehicle_category);
}

void toMsg(
  const openscenario_msgs::VehicleParameters & proto, openscenario_msgs::msg::VehicleParameters & p)
{
  toMsg(proto.axles(), p.axles);
  toMsg(proto.bounding_box(), p.bounding_box);
  toMsg(proto.performance(), p.performance);
  // toMsg(proto.property(), p.property);
  p.name = proto.name();
  p.vehicle_category = proto.vehicle_category();
}

void toProto(
  const openscenario_msgs::msg::PedestrianParameters & p,
  openscenario_msgs::PedestrianParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  proto.set_name(p.name);
  proto.set_pedestrian_category(p.pedestrian_category);
}

void toMsg(
  const openscenario_msgs::PedestrianParameters & proto,
  openscenario_msgs::msg::PedestrianParameters & p)
{
  p.name = proto.name();
  p.pedestrian_category = proto.pedestrian_category();
  toMsg(proto.bounding_box(), p.bounding_box);
}

void toProto(
  const openscenario_msgs::msg::ActionStatus & s, openscenario_msgs::ActionStatus & proto)
{
  proto.set_current_action(s.current_action);
  toProto(s.twist, *proto.mutable_twist());
  toProto(s.accel, *proto.mutable_accel());
}

void toMsg(const openscenario_msgs::ActionStatus & proto, openscenario_msgs::msg::ActionStatus & s)
{
  s.current_action = proto.current_action();
  toMsg(proto.twist(), s.twist);
  toMsg(proto.accel(), s.accel);
}

void toProto(
  const openscenario_msgs::msg::LaneletPose & pose, openscenario_msgs::LaneletPose & proto)
{
  proto.set_lanlet_id(pose.lanelet_id);
  proto.set_s(pose.s);
  proto.set_offset(pose.offset);
  toProto(pose.rpy, *proto.mutable_rpy());
}

void toMsg(const openscenario_msgs::LaneletPose & proto, openscenario_msgs::msg::LaneletPose & pose)
{
  pose.lanelet_id = proto.lanlet_id();
  pose.s = proto.s();
  pose.offset = proto.offset();
  toMsg(proto.rpy(), pose.rpy);
}

void toProto(const openscenario_msgs::msg::EntityType & type, openscenario_msgs::EntityType & proto)
{
  if (type.type == openscenario_msgs::msg::EntityType::EGO) {
    proto = openscenario_msgs::EntityType::EGO;
    return;
  } else if (type.type == openscenario_msgs::msg::EntityType::VEHICLE) {
    proto = openscenario_msgs::EntityType::VEHICLE;
    return;
  } else if (type.type == openscenario_msgs::msg::EntityType::PEDESTRIAN) {
    proto = openscenario_msgs::EntityType::PEDESTRIAN;
    return;
  }
  std::string message = "type of the Entity Type is inavlid!\ntype is " + std::to_string(type.type);
  THROW_PROTOBUF_PARAMETER_ERROR(message);
}

void toMsg(const openscenario_msgs::EntityType & proto, openscenario_msgs::msg::EntityType & type)
{
  if (proto == openscenario_msgs::EntityType::EGO) {
    type.type = openscenario_msgs::msg::EntityType::EGO;
    return;
  }
  if (proto == openscenario_msgs::EntityType::VEHICLE) {
    type.type = openscenario_msgs::msg::EntityType::VEHICLE;
    return;
  }
  if (proto == openscenario_msgs::EntityType::PEDESTRIAN) {
    type.type = openscenario_msgs::msg::EntityType::PEDESTRIAN;
    return;
  }
  std::string message = "type of the Entity Type is inavlid!";
  THROW_PROTOBUF_PARAMETER_ERROR(message);
}

void toProto(
  const openscenario_msgs::msg::EntityStatus & status, openscenario_msgs::EntityStatus & proto)
{
  openscenario_msgs::EntityType type;
  toProto(status.type, type);
  proto.set_type(type);
  proto.set_time(status.time);
  proto.set_name(status.name);
  toProto(status.bounding_box, *proto.mutable_bounding_box());
  toProto(status.action_status, *proto.mutable_action_status());
  toProto(status.pose, *proto.mutable_pose());
  toProto(status.lanelet_pose, *proto.mutable_lanelet_pose());
  proto.set_lanelet_pose_valid(status.lanelet_pose_valid);
}

void toMsg(
  const openscenario_msgs::EntityStatus & proto, openscenario_msgs::msg::EntityStatus & status)
{
  openscenario_msgs::msg::EntityType type;
  toMsg(proto.type(), type);
  status.type = type;
  status.time = proto.time();
  status.name = proto.name();
  toMsg(proto.bounding_box(), status.bounding_box);
  toMsg(proto.action_status(), status.action_status);
  toMsg(proto.pose(), status.pose);
  toMsg(proto.lanelet_pose(), status.lanelet_pose);
  status.lanelet_pose_valid = proto.lanelet_pose_valid();
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
}  // namespace simulation_interface
