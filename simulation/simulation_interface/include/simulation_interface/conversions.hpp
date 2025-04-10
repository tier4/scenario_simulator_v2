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

#ifndef SIMULATION_INTERFACE__CONVERSIONS_HPP_
#define SIMULATION_INTERFACE__CONVERSIONS_HPP_

#include <builtin_interfaces.pb.h>
#include <geometry_msgs.pb.h>
#include <rosgraph_msgs.pb.h>
#include <simulation_api_schema.pb.h>
#include <std_msgs.pb.h>
#include <traffic_simulator_msgs.pb.h>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <rosgraph_msgs/msg/clock.hpp>
#include <simulation_interface/constants.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/action_status.hpp>
#include <traffic_simulator_msgs/msg/axle.hpp>
#include <traffic_simulator_msgs/msg/axles.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/performance.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <vector>
#include <zmqpp/zmqpp.hpp>

namespace zeromq
{
template <typename Proto>
zmqpp::message toZMQ(const Proto & proto)
{
  zmqpp::message msg;
  std::string serialized_str = "";
  proto.SerializeToString(&serialized_str);
  msg << serialized_str;
  return msg;
}

template <typename Proto>
Proto toProto(const zmqpp::message & msg)
{
  std::string serialized_str = msg.get(0);
  Proto proto;
  proto.ParseFromString(serialized_str);
  return proto;
}
}  // namespace zeromq

namespace simulation_interface
{
void toProto(const geometry_msgs::msg::Point & p, geometry_msgs::Point & proto);
void toMsg(const geometry_msgs::Point & proto, geometry_msgs::msg::Point & p);
void toProto(const geometry_msgs::msg::Quaternion & q, geometry_msgs::Quaternion & proto);
void toMsg(const geometry_msgs::Quaternion & proto, geometry_msgs::msg::Quaternion & q);
void toProto(const geometry_msgs::msg::Pose & p, geometry_msgs::Pose & proto);
void toMsg(const geometry_msgs::Pose & proto, geometry_msgs::msg::Pose & p);
void toProto(const geometry_msgs::msg::Vector3 & v, geometry_msgs::Vector3 & proto);
void toMsg(const geometry_msgs::Vector3 & proto, geometry_msgs::msg::Vector3 & v);
void toProto(const geometry_msgs::msg::Twist & t, geometry_msgs::Twist & proto);
void toMsg(const geometry_msgs::Twist & proto, geometry_msgs::msg::Twist & t);
void toProto(const geometry_msgs::msg::Accel & a, geometry_msgs::Accel & proto);
void toMsg(const geometry_msgs::Accel & proto, geometry_msgs::msg::Accel & a);
void toProto(
  const traffic_simulator_msgs::msg::BoundingBox & box,
  traffic_simulator_msgs::BoundingBox & proto);
void toMsg(
  const traffic_simulator_msgs::BoundingBox & proto,
  traffic_simulator_msgs::msg::BoundingBox & box);
void toProto(
  const traffic_simulator_msgs::msg::Performance & performance,
  traffic_simulator_msgs::Performance & proto);
void toMsg(
  const traffic_simulator_msgs::Performance & proto,
  traffic_simulator_msgs::msg::Performance & performance);
void toProto(const traffic_simulator_msgs::msg::Axle & axle, traffic_simulator_msgs::Axle & proto);
void toMsg(const traffic_simulator_msgs::Axle & proto, traffic_simulator_msgs::msg::Axle & axle);
void toProto(
  const traffic_simulator_msgs::msg::Axles & axles, traffic_simulator_msgs::Axles & proto);
void toMsg(const traffic_simulator_msgs::Axles & proto, traffic_simulator_msgs::msg::Axles & axles);
void toProto(
  const traffic_simulator_msgs::msg::VehicleParameters & p,
  traffic_simulator_msgs::VehicleParameters & proto);
void toMsg(
  const traffic_simulator_msgs::VehicleParameters & proto,
  traffic_simulator_msgs::msg::VehicleParameters & p);
void toProto(
  const traffic_simulator_msgs::msg::PedestrianParameters & p,
  traffic_simulator_msgs::PedestrianParameters & proto);
void toMsg(
  const traffic_simulator_msgs::PedestrianParameters & proto,
  traffic_simulator_msgs::msg::PedestrianParameters & p);
void toProto(
  const traffic_simulator_msgs::msg::MiscObjectParameters & p,
  traffic_simulator_msgs::MiscObjectParameters & proto);
void toMsg(
  const traffic_simulator_msgs::MiscObjectParameters & proto,
  traffic_simulator_msgs::msg::MiscObjectParameters & p);
void toProto(
  const traffic_simulator_msgs::msg::ActionStatus & s,
  traffic_simulator_msgs::ActionStatus & proto);
void toMsg(
  const traffic_simulator_msgs::ActionStatus & proto,
  traffic_simulator_msgs::msg::ActionStatus & s);
void toProto(
  const traffic_simulator_msgs::msg::LaneletPose & pose,
  traffic_simulator_msgs::LaneletPose & proto);
void toMsg(
  const traffic_simulator_msgs::LaneletPose & proto,
  traffic_simulator_msgs::msg::LaneletPose & pose);
void toProto(
  const traffic_simulator_msgs::msg::EntityType & type, traffic_simulator_msgs::EntityType & proto);
void toMsg(
  const traffic_simulator_msgs::EntityType & proto, traffic_simulator_msgs::msg::EntityType & type);
void toProto(
  const traffic_simulator_msgs::msg::EntitySubtype & subtype,
  traffic_simulator_msgs::EntitySubtype & proto);
void toMsg(
  const traffic_simulator_msgs::EntitySubtype & proto,
  traffic_simulator_msgs::msg::EntitySubtype & subtype);
void toProto(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  simulation_api_schema::EntityStatus & proto);
void toProto(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  traffic_simulator_msgs::EntityStatus & proto);
void toMsg(
  const traffic_simulator_msgs::EntityStatus & proto,
  traffic_simulator_msgs::msg::EntityStatus & status);
void toMsg(
  const simulation_api_schema::EntityStatus & proto,
  traffic_simulator_msgs::msg::EntityStatus & status);
void toProto(
  const builtin_interfaces::msg::Duration & duration, builtin_interfaces::Duration & proto);
void toMsg(
  const builtin_interfaces::Duration & proto, builtin_interfaces::msg::Duration & duration);
void toProto(const builtin_interfaces::msg::Time & time, builtin_interfaces::Time & proto);
void toMsg(const builtin_interfaces::Time & proto, builtin_interfaces::msg::Time & time);
void toProto(const rosgraph_msgs::msg::Clock & time, rosgraph_msgs::Clock & proto);
void toMsg(const rosgraph_msgs::Clock & proto, rosgraph_msgs::msg::Clock & time);
void toProto(const std_msgs::msg::Header & header, std_msgs::Header & proto);
void toMsg(const std_msgs::Header & proto, std_msgs::msg::Header & header);

#define DEFINE_CONVERSION(PACKAGE, TYPENAME)                               \
  auto toProto(const PACKAGE::msg::TYPENAME &, PACKAGE::TYPENAME &)->void; \
  auto toMsg(const PACKAGE::TYPENAME &, PACKAGE::msg::TYPENAME &)->void

DEFINE_CONVERSION(autoware_control_msgs, Lateral);
DEFINE_CONVERSION(autoware_control_msgs, Longitudinal);
DEFINE_CONVERSION(autoware_control_msgs, Control);
DEFINE_CONVERSION(autoware_vehicle_msgs, GearCommand);

#undef DEFINE_CONVERSION

auto toProto(
  const std::tuple<autoware_control_msgs::msg::Control, autoware_vehicle_msgs::msg::GearCommand> &,
  traffic_simulator_msgs::VehicleCommand &) -> void;

template <typename TrafficLightBulbMessageType>
auto toMsg(
  const simulation_api_schema::TrafficLight & proto,
  TrafficLightBulbMessageType & traffic_light_bulb_state) -> void
{
  using namespace simulation_api_schema;

  auto convert_color = [](auto color) constexpr
  {
    switch (color) {
      case TrafficLight_Color_RED:
        return TrafficLightBulbMessageType::RED;
      case TrafficLight_Color_AMBER:
        return TrafficLightBulbMessageType::AMBER;
      case TrafficLight_Color_GREEN:
        return TrafficLightBulbMessageType::GREEN;
      case TrafficLight_Color_WHITE:
        return TrafficLightBulbMessageType::WHITE;
      default:
        return TrafficLightBulbMessageType::UNKNOWN;
    }
  };

  auto convert_shape = [](auto shape) constexpr
  {
    switch (shape) {
      case TrafficLight_Shape_CIRCLE:
        return TrafficLightBulbMessageType::CIRCLE;
      case TrafficLight_Shape_LEFT_ARROW:
        return TrafficLightBulbMessageType::LEFT_ARROW;
      case TrafficLight_Shape_RIGHT_ARROW:
        return TrafficLightBulbMessageType::RIGHT_ARROW;
      case TrafficLight_Shape_UP_ARROW:
        return TrafficLightBulbMessageType::UP_ARROW;
        /// @note Enums below are not supported yet in some platforms. I temporarily disabled them
        //  case TrafficLight_Shape_UP_LEFT_ARROW:
        //    return TrafficLightBulbMessageType::UP_LEFT_ARROW;
        //  case TrafficLight_Shape_UP_RIGHT_ARROW:
        //    return TrafficLightBulbMessageType::UP_RIGHT_ARROW;
      case TrafficLight_Shape_DOWN_ARROW:
        return TrafficLightBulbMessageType::DOWN_ARROW;
      case TrafficLight_Shape_DOWN_LEFT_ARROW:
        return TrafficLightBulbMessageType::DOWN_LEFT_ARROW;
      case TrafficLight_Shape_DOWN_RIGHT_ARROW:
        return TrafficLightBulbMessageType::DOWN_RIGHT_ARROW;
      case TrafficLight_Shape_CROSS:
        return TrafficLightBulbMessageType::CROSS;
      default:
        return TrafficLightBulbMessageType::UNKNOWN;
    }
  };

  auto convert_status = [](auto status) constexpr
  {
    switch (status) {
      case TrafficLight_Status_SOLID_OFF:
        return TrafficLightBulbMessageType::SOLID_OFF;
      case TrafficLight_Status_SOLID_ON:
        return TrafficLightBulbMessageType::SOLID_ON;
      case TrafficLight_Status_FLASHING:
        return TrafficLightBulbMessageType::FLASHING;
      default:
        return TrafficLightBulbMessageType::UNKNOWN;
    }
  };

  traffic_light_bulb_state.status = convert_status(proto.status());
  traffic_light_bulb_state.shape = convert_shape(proto.shape());
  traffic_light_bulb_state.color = convert_color(proto.color());
  traffic_light_bulb_state.confidence = proto.confidence();
}

auto toProtobufMessage(const traffic_simulator_msgs::msg::Vertex &)
  -> traffic_simulator_msgs::Vertex;

auto toROS2Message(const traffic_simulator_msgs::Vertex &) -> traffic_simulator_msgs::msg::Vertex;

auto toProtobufMessage(const traffic_simulator_msgs::msg::Polyline &)
  -> traffic_simulator_msgs::Polyline;

auto toROS2Message(const traffic_simulator_msgs::Polyline &)
  -> traffic_simulator_msgs::msg::Polyline;

auto toProtobufMessage(const traffic_simulator_msgs::msg::PolylineTrajectory &)
  -> traffic_simulator_msgs::PolylineTrajectory;

auto toROS2Message(const traffic_simulator_msgs::PolylineTrajectory &)
  -> traffic_simulator_msgs::msg::PolylineTrajectory;
}  // namespace simulation_interface

#endif  // SIMULATION_INTERFACE__CONVERSIONS_HPP_
