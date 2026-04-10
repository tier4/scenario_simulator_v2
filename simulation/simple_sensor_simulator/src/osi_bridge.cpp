// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include <simple_sensor_simulator/osi_bridge.hpp>

namespace simple_sensor_simulator
{
namespace osi_bridge
{
auto toProtoEntityStatus(const osi_interface::EntityData & entity)
  -> simulation_api_schema::EntityStatus
{
  simulation_api_schema::EntityStatus status;
  status.set_name(entity.name);
  status.set_time(0.0);  // timestamp is global, not per-entity in OSI

  // Type
  switch (entity.type) {
    case osi_interface::EntityType::EGO:
      status.mutable_type()->set_type(traffic_simulator_msgs::EntityType::EGO);
      break;
    case osi_interface::EntityType::VEHICLE:
      status.mutable_type()->set_type(traffic_simulator_msgs::EntityType::VEHICLE);
      break;
    case osi_interface::EntityType::PEDESTRIAN:
      status.mutable_type()->set_type(traffic_simulator_msgs::EntityType::PEDESTRIAN);
      break;
    case osi_interface::EntityType::MISC_OBJECT:
      status.mutable_type()->set_type(traffic_simulator_msgs::EntityType::MISC_OBJECT);
      break;
  }

  // Subtype
  status.mutable_subtype()->set_value(
    static_cast<traffic_simulator_msgs::EntitySubtype::Enum>(entity.subtype));

  // Pose
  status.mutable_pose()->mutable_position()->set_x(entity.pose.x);
  status.mutable_pose()->mutable_position()->set_y(entity.pose.y);
  status.mutable_pose()->mutable_position()->set_z(entity.pose.z);
  status.mutable_pose()->mutable_orientation()->set_x(entity.pose.qx);
  status.mutable_pose()->mutable_orientation()->set_y(entity.pose.qy);
  status.mutable_pose()->mutable_orientation()->set_z(entity.pose.qz);
  status.mutable_pose()->mutable_orientation()->set_w(entity.pose.qw);

  // Action status
  status.mutable_action_status()->mutable_twist()->mutable_linear()->set_x(entity.twist.linear_x);
  status.mutable_action_status()->mutable_twist()->mutable_linear()->set_y(entity.twist.linear_y);
  status.mutable_action_status()->mutable_twist()->mutable_linear()->set_z(entity.twist.linear_z);
  status.mutable_action_status()->mutable_twist()->mutable_angular()->set_x(entity.twist.angular_x);
  status.mutable_action_status()->mutable_twist()->mutable_angular()->set_y(entity.twist.angular_y);
  status.mutable_action_status()->mutable_twist()->mutable_angular()->set_z(entity.twist.angular_z);
  status.mutable_action_status()->mutable_accel()->mutable_linear()->set_x(entity.accel.linear_x);
  status.mutable_action_status()->mutable_accel()->mutable_linear()->set_y(entity.accel.linear_y);
  status.mutable_action_status()->mutable_accel()->mutable_linear()->set_z(entity.accel.linear_z);

  return status;
}

auto toRosMsgEntityStatus(const osi_interface::EntityData & entity)
  -> traffic_simulator_msgs::msg::EntityStatus
{
  traffic_simulator_msgs::msg::EntityStatus msg;
  msg.name = entity.name;

  // Pose
  msg.pose.position.x = entity.pose.x;
  msg.pose.position.y = entity.pose.y;
  msg.pose.position.z = entity.pose.z;
  msg.pose.orientation.x = entity.pose.qx;
  msg.pose.orientation.y = entity.pose.qy;
  msg.pose.orientation.z = entity.pose.qz;
  msg.pose.orientation.w = entity.pose.qw;

  // Action status
  msg.action_status.twist.linear.x = entity.twist.linear_x;
  msg.action_status.twist.linear.y = entity.twist.linear_y;
  msg.action_status.twist.linear.z = entity.twist.linear_z;
  msg.action_status.twist.angular.x = entity.twist.angular_x;
  msg.action_status.twist.angular.y = entity.twist.angular_y;
  msg.action_status.twist.angular.z = entity.twist.angular_z;
  msg.action_status.accel.linear.x = entity.accel.linear_x;
  msg.action_status.accel.linear.y = entity.accel.linear_y;
  msg.action_status.accel.linear.z = entity.accel.linear_z;

  // Bounding box
  msg.bounding_box.center.x = entity.bounding_box.center_x;
  msg.bounding_box.center.y = entity.bounding_box.center_y;
  msg.bounding_box.center.z = entity.bounding_box.center_z;
  msg.bounding_box.dimensions.x = entity.bounding_box.length;
  msg.bounding_box.dimensions.y = entity.bounding_box.width;
  msg.bounding_box.dimensions.z = entity.bounding_box.height;

  return msg;
}

auto fromRosMsgEntityStatus(const traffic_simulator_msgs::msg::EntityStatus & msg)
  -> osi_interface::EntityData
{
  osi_interface::EntityData entity;
  entity.name = msg.name;

  entity.pose.x = msg.pose.position.x;
  entity.pose.y = msg.pose.position.y;
  entity.pose.z = msg.pose.position.z;
  entity.pose.qx = msg.pose.orientation.x;
  entity.pose.qy = msg.pose.orientation.y;
  entity.pose.qz = msg.pose.orientation.z;
  entity.pose.qw = msg.pose.orientation.w;

  entity.twist.linear_x = msg.action_status.twist.linear.x;
  entity.twist.linear_y = msg.action_status.twist.linear.y;
  entity.twist.linear_z = msg.action_status.twist.linear.z;
  entity.twist.angular_x = msg.action_status.twist.angular.x;
  entity.twist.angular_y = msg.action_status.twist.angular.y;
  entity.twist.angular_z = msg.action_status.twist.angular.z;

  entity.accel.linear_x = msg.action_status.accel.linear.x;
  entity.accel.linear_y = msg.action_status.accel.linear.y;
  entity.accel.linear_z = msg.action_status.accel.linear.z;

  entity.bounding_box.center_x = msg.bounding_box.center.x;
  entity.bounding_box.center_y = msg.bounding_box.center.y;
  entity.bounding_box.center_z = msg.bounding_box.center.z;
  entity.bounding_box.length = msg.bounding_box.dimensions.x;
  entity.bounding_box.width = msg.bounding_box.dimensions.y;
  entity.bounding_box.height = msg.bounding_box.dimensions.z;

  return entity;
}

auto makeDefaultVehicleParameters(const osi_interface::EntityData & entity)
  -> traffic_simulator_msgs::msg::VehicleParameters
{
  traffic_simulator_msgs::msg::VehicleParameters params;
  params.name = entity.name;

  // Bounding box from entity data
  params.bounding_box.center.x = entity.bounding_box.center_x;
  params.bounding_box.center.y = entity.bounding_box.center_y;
  params.bounding_box.center.z = entity.bounding_box.center_z;
  params.bounding_box.dimensions.x = entity.bounding_box.length;
  params.bounding_box.dimensions.y = entity.bounding_box.width;
  params.bounding_box.dimensions.z = entity.bounding_box.height;

  // Default performance parameters
  params.performance.max_speed = 50.0;
  params.performance.max_acceleration = 10.0;
  params.performance.max_deceleration = 10.0;
  params.performance.max_acceleration_rate = 10.0;
  params.performance.max_deceleration_rate = 10.0;

  // Default axle parameters (derived from bounding box)
  const double wheel_base = entity.bounding_box.length * 0.6;
  params.axles.front_axle.position_x = wheel_base / 2.0;
  params.axles.front_axle.position_z = 0.0;
  params.axles.front_axle.wheel_diameter = 0.65;
  params.axles.front_axle.track_width = entity.bounding_box.width * 0.85;
  params.axles.front_axle.max_steering = 0.5;
  params.axles.rear_axle.position_x = -wheel_base / 2.0;
  params.axles.rear_axle.position_z = 0.0;
  params.axles.rear_axle.wheel_diameter = 0.65;
  params.axles.rear_axle.track_width = entity.bounding_box.width * 0.85;
  params.axles.rear_axle.max_steering = 0.0;

  return params;
}

auto toProtoTrafficLightsRequest(const std::vector<osi_interface::TrafficSignalGroup> & signals)
  -> simulation_api_schema::UpdateTrafficLightsRequest
{
  simulation_api_schema::UpdateTrafficLightsRequest req;
  for (const auto & signal : signals) {
    auto * proto_signal = req.add_states();
    proto_signal->set_id(signal.lanelet_id);
    for (const auto & relation_id : signal.relation_ids) {
      proto_signal->add_relation_ids(relation_id);
    }
    for (const auto & bulb : signal.bulbs) {
      auto * proto_bulb = proto_signal->add_traffic_light_status();
      proto_bulb->set_color(static_cast<simulation_api_schema::TrafficLight::Color>(bulb.color));
      proto_bulb->set_shape(static_cast<simulation_api_schema::TrafficLight::Shape>(bulb.shape));
      proto_bulb->set_status(static_cast<simulation_api_schema::TrafficLight::Status>(bulb.status));
      proto_bulb->set_confidence(bulb.confidence);
    }
  }
  return req;
}

}  // namespace osi_bridge
}  // namespace simple_sensor_simulator
