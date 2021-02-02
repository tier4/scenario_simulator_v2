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

#include <xmlrpc_interface/conversions.hpp>

#include <string>
#include <vector>

namespace xmlrpc_interface
{
void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeResponse & to)
{
  to = simulation_api_schema::InitializeResponse();
  to.mutable_result()->set_success(xmlrpc_interface::getXmlValue<bool>(from, key::success));
  to.mutable_result()->set_description(
    xmlrpc_interface::getXmlValue<std::string>(
      from,
      key::description));
}

void fromProto(const simulation_api_schema::InitializeResponse & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key::success] = from.result().success();
  to[key::description] = from.result().description();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::InitializeRequest & to)
{
  to = simulation_api_schema::InitializeRequest();
  to.set_realtime_factor(xmlrpc_interface::getXmlValue<double>(from, key::realtime_factor));
  to.set_step_time(xmlrpc_interface::getXmlValue<double>(from, key::step_time));
}

void fromProto(const simulation_api_schema::InitializeRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key::realtime_factor] = from.realtime_factor();
  to[key::step_time] = from.step_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameRequest & to)
{
  to = simulation_api_schema::UpdateFrameRequest();
  to.set_current_time(from[key::current_time]);
}

void fromProto(const simulation_api_schema::UpdateFrameRequest & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key::current_time] = from.current_time();
}

void toProto(const XmlRpc::XmlRpcValue & from, simulation_api_schema::UpdateFrameResponse & to)
{
  to = simulation_api_schema::UpdateFrameResponse();
  to.mutable_result()->set_success(from[key::success]);
  to.mutable_result()->set_description(from[key::description]);
}

void fromProto(const simulation_api_schema::UpdateFrameResponse & from, XmlRpc::XmlRpcValue & to)
{
  to = XmlRpc::XmlRpcValue();
  to[key::success] = from.result().success();
  to[key::description] = from.result().description();
}

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
  const openscenario_msgs::msg::BoundingBox & box,
  openscenario_msgs::BoundingBox & proto)
{
  toProto(box.center, *proto.mutable_center());
  toProto(box.dimensions, *proto.mutable_dimensions());
}

void toMsg(
  const openscenario_msgs::BoundingBox & proto,
  openscenario_msgs::msg::BoundingBox & box)
{
  toMsg(proto.center(), box.center);
  toMsg(proto.dimensions(), box.dimensions);
}

void toProto(
  const openscenario_msgs::msg::Performance & performance,
  openscenario_msgs::Performance & proto)
{
  proto.set_max_acceleration(performance.max_acceleration);
  proto.set_max_deceleration(performance.max_deceleration);
  proto.set_max_speed(performance.max_speed);
}

void toMsg(
  const openscenario_msgs::Performance & proto,
  openscenario_msgs::msg::Performance & performance)
{
  performance.max_acceleration = proto.max_acceleration();
  performance.max_deceleration = proto.max_deceleration();
  performance.max_speed = proto.max_speed();
}

void toProto(
  const openscenario_msgs::msg::Axle & axle,
  openscenario_msgs::Axle & proto)
{
  proto.set_position_x(axle.position_x);
  proto.set_position_z(axle.position_z);
  proto.set_track_width(axle.track_width);
  proto.set_wheel_diameter(axle.wheel_diameter);
  proto.set_max_steering(axle.max_steering);
}

void toMsg(
  const openscenario_msgs::Axle & proto,
  openscenario_msgs::msg::Axle & axle)
{
  axle.position_x = proto.position_x();
  axle.position_z = proto.position_z();
  axle.track_width = proto.track_width();
  axle.wheel_diameter = proto.wheel_diameter();
  axle.max_steering = proto.max_steering();
}

void toProto(
  const openscenario_msgs::msg::Axles & axles,
  openscenario_msgs::Axles & proto)
{
  toProto(axles.front_axle, *proto.mutable_front_axle());
  toProto(axles.rear_axle, *proto.mutable_rear_axle());
}

void toMsg(
  const openscenario_msgs::Axles & proto,
  openscenario_msgs::msg::Axles & axles)
{
  toMsg(proto.front_axle(), axles.front_axle);
  toMsg(proto.rear_axle(), axles.rear_axle);
}

void toProto(
  const openscenario_msgs::msg::Property & p,
  openscenario_msgs::Property & proto)
{
  proto.set_is_ego(p.is_ego);
}

void toMsg(
  const openscenario_msgs::Property & proto,
  openscenario_msgs::msg::Property & p)
{
  p.is_ego = proto.is_ego();
}
}  // namespace xmlrpc_interface
