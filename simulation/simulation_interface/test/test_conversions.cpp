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

#include <geometry_msgs.pb.h>
#include <gtest/gtest.h>

#include "expect_equal_macros.hpp"
/**
 * @brief Test cases
 */

TEST(Conversion, Point)
{
  geometry_msgs::Point proto;
  geometry_msgs::msg::Point p;
  p.x = 1.0;
  p.y = 2;
  p.z = 3.1;
  simulation_interface::toProto(p, proto);
  EXPECT_POINT_EQ(p, proto);
  p = geometry_msgs::msg::Point();
  EXPECT_DOUBLE_EQ(p.x, 0);
  simulation_interface::toMsg(proto, p);
  EXPECT_POINT_EQ(p, proto);
}

TEST(Conversion, Quaternion)
{
  geometry_msgs::Quaternion proto;
  geometry_msgs::msg::Quaternion q;
  q.x = 1.0;
  q.y = 2;
  q.z = 3.1;
  q.w = -10;
  simulation_interface::toProto(q, proto);
  EXPECT_QUATERNION_EQ(q, proto);
  q = geometry_msgs::msg::Quaternion();
  EXPECT_DOUBLE_EQ(q.x, 0);
  simulation_interface::toMsg(proto, q);
  EXPECT_QUATERNION_EQ(q, proto);
}

TEST(Conversion, Pose)
{
  geometry_msgs::Pose proto;
  geometry_msgs::msg::Pose p;
  p.position.x = 1.0;
  p.position.y = 2;
  p.position.z = 3.1;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 1;
  simulation_interface::toProto(p, proto);
  EXPECT_POSE_EQ(p, proto);
  p = geometry_msgs::msg::Pose();
  EXPECT_DOUBLE_EQ(p.position.x, 0);
  simulation_interface::toMsg(proto, p);
  EXPECT_POSE_EQ(p, proto);
}

TEST(Conversion, Vector)
{
  geometry_msgs::Vector3 proto;
  geometry_msgs::msg::Vector3 vec;
  vec.x = 1;
  vec.y = 2;
  vec.z = 3.0;
  simulation_interface::toProto(vec, proto);
  EXPECT_VECTOR3_EQ(vec, proto);
  vec = geometry_msgs::msg::Vector3();
  EXPECT_DOUBLE_EQ(vec.x, 0);
  simulation_interface::toMsg(proto, vec);
  EXPECT_VECTOR3_EQ(vec, proto);
}

TEST(Conversion, Twist)
{
  geometry_msgs::Twist proto;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1;
  twist.linear.y = 2;
  twist.linear.z = 3.0;
  simulation_interface::toProto(twist, proto);
  EXPECT_TWIST_EQ(twist, proto);
  twist = geometry_msgs::msg::Twist();
  EXPECT_DOUBLE_EQ(twist.linear.x, 0);
  simulation_interface::toMsg(proto, twist);
  EXPECT_TWIST_EQ(twist, proto);
}

TEST(Conversion, Accel)
{
  geometry_msgs::Accel proto;
  geometry_msgs::msg::Accel accel;
  accel.linear.x = 1;
  accel.linear.y = 2;
  accel.linear.z = 3.0;
  simulation_interface::toProto(accel, proto);
  EXPECT_ACCEL_EQ(accel, proto);
  accel = geometry_msgs::msg::Accel();
  EXPECT_DOUBLE_EQ(accel.linear.x, 0);
  simulation_interface::toMsg(proto, accel);
  EXPECT_ACCEL_EQ(accel, proto);
}

TEST(Conversion, Performance)
{
  traffic_simulator_msgs::Performance proto;
  traffic_simulator_msgs::msg::Performance performance;
  performance.max_speed = 10;
  performance.max_deceleration = 3;
  simulation_interface::toProto(performance, proto);
  EXPECT_PERFORMANCE_EQ(performance, proto);
  EXPECT_DOUBLE_EQ(performance.max_acceleration, proto.max_acceleration());
  EXPECT_DOUBLE_EQ(performance.max_deceleration, proto.max_deceleration());
  EXPECT_DOUBLE_EQ(performance.max_speed, proto.max_speed());
  performance = traffic_simulator_msgs::msg::Performance();
  EXPECT_DOUBLE_EQ(performance.max_speed, 30.0);
  simulation_interface::toMsg(proto, performance);
  EXPECT_PERFORMANCE_EQ(performance, proto);
}

TEST(Conversion, Axle)
{
  traffic_simulator_msgs::Axle proto;
  traffic_simulator_msgs::msg::Axle axle;
  axle.max_steering = 30;
  axle.position_x = 3;
  axle.position_z = 14;
  axle.track_width = -10;
  axle.wheel_diameter = 53;
  simulation_interface::toProto(axle, proto);
  EXPECT_AXLE_EQ(axle, proto);
  axle = traffic_simulator_msgs::msg::Axle();
  EXPECT_DOUBLE_EQ(axle.max_steering, 0);
  simulation_interface::toMsg(proto, axle);
  EXPECT_AXLE_EQ(axle, proto);
}

TEST(Conversion, Axles)
{
  traffic_simulator_msgs::Axles proto;
  traffic_simulator_msgs::msg::Axles axles;
  axles.front_axle.max_steering = 3;
  axles.front_axle.position_x = 35;
  axles.front_axle.position_z = 234;
  axles.front_axle.track_width = 1;
  axles.front_axle.wheel_diameter = 123;
  axles.rear_axle.max_steering = 13;
  axles.rear_axle.position_x = 3;
  axles.rear_axle.position_z = 23;
  axles.rear_axle.track_width = 14;
  axles.rear_axle.wheel_diameter = 122;
  simulation_interface::toProto(axles, proto);
  EXPECT_AXLES_EQ(axles, proto);
  axles = traffic_simulator_msgs::msg::Axles();
  EXPECT_DOUBLE_EQ(axles.front_axle.max_steering, 0);
  simulation_interface::toMsg(proto, axles);
  EXPECT_AXLES_EQ(axles, proto);
}

TEST(Conversion, BoundingBox)
{
  traffic_simulator_msgs::BoundingBox proto;
  traffic_simulator_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  simulation_interface::toProto(box, proto);
  EXPECT_BOUNDING_BOX_EQ(box, proto);
  box = traffic_simulator_msgs::msg::BoundingBox();
  EXPECT_DOUBLE_EQ(box.center.x, 0);
  simulation_interface::toMsg(proto, box);
  EXPECT_BOUNDING_BOX_EQ(box, proto);
}

TEST(Conversion, VehicleParameters)
{
  traffic_simulator_msgs::VehicleParameters proto;
  traffic_simulator_msgs::msg::VehicleParameters p;
  p.name = "foo";
  traffic_simulator_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  traffic_simulator_msgs::msg::Performance performance;
  performance.max_speed = 10;
  performance.max_deceleration = 3;
  p.performance = performance;
  traffic_simulator_msgs::msg::Axle axle;
  axle.max_steering = 30;
  axle.position_x = 3;
  axle.position_z = 14;
  axle.track_width = -10;
  axle.wheel_diameter = 53;
  p.axles.front_axle = axle;
  p.axles.rear_axle = axle;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_VEHICLE_PARAMETERS_EQ(p, proto);
  p = traffic_simulator_msgs::msg::VehicleParameters();
  EXPECT_DOUBLE_EQ(p.bounding_box.dimensions.x, 0);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_VEHICLE_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, PedestrianParameters)
{
  traffic_simulator_msgs::PedestrianParameters proto;
  traffic_simulator_msgs::msg::PedestrianParameters p;
  p.name = "foo";
  traffic_simulator_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(p, proto);
  p = traffic_simulator_msgs::msg::PedestrianParameters();
  EXPECT_DOUBLE_EQ(p.bounding_box.dimensions.x, 0);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, MiscObjectParameters)
{
  traffic_simulator_msgs::MiscObjectParameters proto;
  traffic_simulator_msgs::msg::MiscObjectParameters p;
  traffic_simulator_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(p, proto);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, ActionStatus)
{
  traffic_simulator_msgs::ActionStatus proto;
  traffic_simulator_msgs::msg::ActionStatus action;
  action.current_action = "test";
  action.twist.linear.x = 1.0;
  action.twist.linear.y = 2.0;
  action.twist.linear.z = 3.0;
  action.twist.angular.x = -20;
  action.twist.angular.y = -4.2;
  action.twist.angular.z = 9;
  action.accel.linear.x = 3.0;
  action.accel.linear.y = 908;
  action.accel.linear.z = 987.0;
  action.accel.angular.x = 0.3;
  action.accel.angular.y = 0.5;
  action.accel.angular.z = 98;
  simulation_interface::toProto(action, proto);
  EXPECT_ACTION_STATUS_EQ(action, proto);
  action = traffic_simulator_msgs::msg::ActionStatus();
  EXPECT_DOUBLE_EQ(action.twist.linear.x, 0);
  simulation_interface::toMsg(proto, action);
  EXPECT_ACTION_STATUS_EQ(action, proto);
}

TEST(Conversion, EntityStatus)
{
  traffic_simulator_msgs::EntityStatus proto;
  traffic_simulator_msgs::msg::EntityStatus status;
  status.name = "test";
  status.time = 3.0;
  traffic_simulator_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  status.bounding_box = box;
  traffic_simulator_msgs::msg::ActionStatus action;
  action.current_action = "test";
  action.twist.linear.x = 1.0;
  action.twist.linear.y = 2.0;
  action.twist.linear.z = 3.0;
  action.twist.angular.x = -20;
  action.twist.angular.y = -4.2;
  action.twist.angular.z = 9;
  action.accel.linear.x = 3.0;
  action.accel.linear.y = 908;
  action.accel.linear.z = 987.0;
  action.accel.angular.x = 0.3;
  action.accel.angular.y = 0.5;
  action.accel.angular.z = 98;
  status.action_status = action;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 4.0;
  pose.position.y = 1.2;
  pose.position.z = 5.1;
  pose.orientation.x = 0.3;
  pose.orientation.y = 8.3;
  pose.orientation.z = 9.3;
  pose.orientation.w = 10.2;
  status.pose = pose;
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = 23;
  lanelet_pose.s = 1.0;
  lanelet_pose.offset = 3.5;
  lanelet_pose.rpy.x = 3.4;
  lanelet_pose.rpy.y = 5.1;
  lanelet_pose.rpy.z = 1.3;
  status.lanelet_pose = lanelet_pose;
  status.lanelet_pose_valid = false;
  simulation_interface::toProto(status, proto);
  EXPECT_ENTITY_STATUS_EQ(status, proto);
  status = traffic_simulator_msgs::msg::EntityStatus();
  EXPECT_TRUE(status.lanelet_pose_valid);
  EXPECT_FALSE(proto.lanelet_pose_valid());
  simulation_interface::toMsg(proto, status);
  EXPECT_ENTITY_STATUS_EQ(status, proto);
  EXPECT_FALSE(status.lanelet_pose_valid);
}

TEST(Conversion, SentEntityStatus)
{
  simulation_api_schema::EntityStatus proto;
  traffic_simulator_msgs::msg::EntityStatus status;
  status.name = "test";
  status.time = 3.0;
  traffic_simulator_msgs::msg::ActionStatus action;
  action.current_action = "test";
  action.twist.linear.x = 1.0;
  action.twist.linear.y = 2.0;
  action.twist.linear.z = 3.0;
  action.twist.angular.x = -20;
  action.twist.angular.y = -4.2;
  action.twist.angular.z = 9;
  action.accel.linear.x = 3.0;
  action.accel.linear.y = 908;
  action.accel.linear.z = 987.0;
  action.accel.angular.x = 0.3;
  action.accel.angular.y = 0.5;
  action.accel.angular.z = 98;
  status.action_status = action;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 4.0;
  pose.position.y = 1.2;
  pose.position.z = 5.1;
  pose.orientation.x = 0.3;
  pose.orientation.y = 8.3;
  pose.orientation.z = 9.3;
  pose.orientation.w = 10.2;
  status.pose = pose;
  simulation_interface::toProto(status, proto);
  EXPECT_SENT_ENTITY_STATUS_EQ(status, proto);
  status = traffic_simulator_msgs::msg::EntityStatus();
  simulation_interface::toMsg(proto, status);
  EXPECT_SENT_ENTITY_STATUS_EQ(status, proto);
}

TEST(Conversion, Time)
{
  builtin_interfaces::Time proto;
  builtin_interfaces::msg::Time msg;
  msg.nanosec = 1;
  msg.sec = 2;
  simulation_interface::toProto(msg, proto);
  EXPECT_TIME_EQ(msg, proto);
  msg.nanosec = 0;
  msg.sec = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_TIME_EQ(msg, proto);
}

TEST(Conversion, Duration)
{
  builtin_interfaces::Duration proto;
  builtin_interfaces::msg::Duration msg;
  msg.nanosec = 1;
  msg.sec = 2;
  simulation_interface::toProto(msg, proto);
  EXPECT_DURATION_EQ(msg, proto);
  msg.nanosec = 0;
  msg.sec = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_DURATION_EQ(msg, proto);
}

TEST(Conversion, Header)
{
  std_msgs::Header proto;
  std_msgs::msg::Header msg;
  msg.frame_id = "base_link";
  msg.stamp.nanosec = 4;
  msg.stamp.sec = 1;
  simulation_interface::toProto(msg, proto);
  EXPECT_HEADER_EQ(msg, proto);
  msg.frame_id = "";
  msg.stamp.nanosec = 0;
  msg.stamp.sec = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_HEADER_EQ(msg, proto);
}

TEST(Conversion, Clock)
{
  rosgraph_msgs::msg::Clock msg;
  rosgraph_msgs::Clock proto;
  msg.clock.nanosec = 12;
  msg.clock.sec = 11;
  simulation_interface::toProto(msg, proto);
  EXPECT_CLOCK_EQ(msg, proto);
  msg = rosgraph_msgs::msg::Clock();
  EXPECT_EQ(msg.clock.nanosec, static_cast<uint32_t>(0));
  simulation_interface::toMsg(proto, msg);
  EXPECT_CLOCK_EQ(msg, proto);
}

TEST(Conversion, Control)
{
  autoware_control_msgs::Control proto;
  autoware_control_msgs::msg::Control msg;
  msg.longitudinal.acceleration = 3;
  msg.lateral.steering_tire_angle = 1.4;
  msg.lateral.steering_tire_rotation_rate = 13.4;
  msg.longitudinal.velocity = 11.3;
  simulation_interface::toProto(msg, proto);
  EXPECT_CONTROL_COMMAND_EQ(msg, proto);
  msg.longitudinal.acceleration = 0;
  msg.lateral.steering_tire_angle = 0;
  msg.lateral.steering_tire_rotation_rate = 0;
  msg.longitudinal.velocity = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_CONTROL_COMMAND_EQ(msg, proto);
}

TEST(Conversion, EntityType)
{
  traffic_simulator_msgs::EntityType proto;
  traffic_simulator_msgs::msg::EntityType msg;
  msg.type = msg.VEHICLE;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_VEHICLE);
  msg.type = msg.MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_MISC_OBJECT);
  proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_VEHICLE);
  msg.type = msg.EGO;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::VEHICLE);
  msg.type = msg.VEHICLE;
  proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::EGO);
  msg.type = msg.VEHICLE;
  proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_PEDESTRIAN);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
  proto.set_type(traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_MISC_OBJECT);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT);
}

TEST(Conversion, EntitySubtype)
{
  traffic_simulator_msgs::EntitySubtype proto;
  traffic_simulator_msgs::msg::EntitySubtype msg;
  msg.value = msg.CAR;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR);
  msg.value = msg.UNKNOWN;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN);
  proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR);
  msg.value = msg.UNKNOWN;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.value, traffic_simulator_msgs::msg::EntitySubtype::CAR);
  msg.value = msg.CAR;
  proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.value, traffic_simulator_msgs::msg::EntitySubtype::CAR);
  msg.value = msg.CAR;
  proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_PEDESTRIAN);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.value, traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN);
  proto.set_value(traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.value, traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN);
}

TEST(Conversion, LaneletPose)
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  traffic_simulator_msgs::LaneletPose proto;
  pose.lanelet_id = 23;
  pose.s = 1.0;
  pose.offset = 3.5;
  pose.rpy.x = 3.4;
  pose.rpy.y = 5.1;
  pose.rpy.z = 1.3;
  EXPECT_NO_THROW(simulation_interface::toProto(pose, proto));
  EXPECT_LANELET_POSE_EQ(pose, proto);
  pose = traffic_simulator_msgs::msg::LaneletPose();
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, pose));
  EXPECT_LANELET_POSE_EQ(pose, proto);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
