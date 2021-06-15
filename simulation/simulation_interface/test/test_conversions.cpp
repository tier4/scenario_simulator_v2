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

#include <geometry_msgs.pb.h>
#include <gtest/gtest.h>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>

#define EXPECT_CONTROL_COMMAND_EQ(msg, proto)                                     \
  EXPECT_DOUBLE_EQ(msg.velocity, proto.velocity());                               \
  EXPECT_DOUBLE_EQ(msg.steering_angle_velocity, proto.steering_angle_velocity()); \
  EXPECT_DOUBLE_EQ(msg.steering_angle, proto.steering_angle());                   \
  EXPECT_DOUBLE_EQ(msg.acceleration, proto.acceleration());

#define EXPECT_HEADER_EQ(msg, proto)                            \
  EXPECT_STREQ(msg.frame_id.c_str(), proto.frame_id().c_str()); \
  EXPECT_EQ(msg.stamp.sec, proto.stamp().sec());                \
  EXPECT_EQ(msg.stamp.nanosec, proto.stamp().nanosec());

TEST(Conversion, ConvertPoint)
{
  geometry_msgs::Point proto;
  geometry_msgs::msg::Point p;
  p.x = 1.0;
  p.y = 2;
  p.z = 3.1;
  simulation_interface::toProto(p, proto);
  EXPECT_DOUBLE_EQ(p.x, proto.x());
  EXPECT_DOUBLE_EQ(p.y, proto.y());
  EXPECT_DOUBLE_EQ(p.z, proto.z());
  p = geometry_msgs::msg::Point();
  EXPECT_DOUBLE_EQ(p.x, 0);
  simulation_interface::toMsg(proto, p);
  EXPECT_DOUBLE_EQ(p.x, proto.x());
  EXPECT_DOUBLE_EQ(p.y, proto.y());
  EXPECT_DOUBLE_EQ(p.z, proto.z());
}

TEST(Conversion, ConvertQuaternion)
{
  geometry_msgs::Quaternion proto;
  geometry_msgs::msg::Quaternion q;
  q.x = 1.0;
  q.y = 2;
  q.z = 3.1;
  q.w = -10;
  simulation_interface::toProto(q, proto);
  EXPECT_DOUBLE_EQ(q.x, proto.x());
  EXPECT_DOUBLE_EQ(q.y, proto.y());
  EXPECT_DOUBLE_EQ(q.z, proto.z());
  EXPECT_DOUBLE_EQ(q.w, proto.w());
  q = geometry_msgs::msg::Quaternion();
  EXPECT_DOUBLE_EQ(q.x, 0);
  simulation_interface::toMsg(proto, q);
  EXPECT_DOUBLE_EQ(q.x, proto.x());
  EXPECT_DOUBLE_EQ(q.y, proto.y());
  EXPECT_DOUBLE_EQ(q.z, proto.z());
  EXPECT_DOUBLE_EQ(q.w, proto.w());
}

TEST(Conversion, ConvertPose)
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
  EXPECT_DOUBLE_EQ(p.position.x, proto.position().x());
  EXPECT_DOUBLE_EQ(p.position.y, proto.position().y());
  EXPECT_DOUBLE_EQ(p.position.z, proto.position().z());
  EXPECT_DOUBLE_EQ(p.orientation.x, proto.orientation().x());
  EXPECT_DOUBLE_EQ(p.orientation.y, proto.orientation().y());
  EXPECT_DOUBLE_EQ(p.orientation.z, proto.orientation().z());
  EXPECT_DOUBLE_EQ(p.orientation.w, proto.orientation().w());
  p = geometry_msgs::msg::Pose();
  EXPECT_DOUBLE_EQ(p.position.x, 0);
  simulation_interface::toMsg(proto, p);
  EXPECT_DOUBLE_EQ(p.position.x, proto.position().x());
  EXPECT_DOUBLE_EQ(p.position.y, proto.position().y());
  EXPECT_DOUBLE_EQ(p.position.z, proto.position().z());
  EXPECT_DOUBLE_EQ(p.orientation.x, proto.orientation().x());
  EXPECT_DOUBLE_EQ(p.orientation.y, proto.orientation().y());
  EXPECT_DOUBLE_EQ(p.orientation.z, proto.orientation().z());
  EXPECT_DOUBLE_EQ(p.orientation.w, proto.orientation().w());
}

TEST(Conversion, ConvertVector)
{
  geometry_msgs::Vector3 proto;
  geometry_msgs::msg::Vector3 vec;
  vec.x = 1;
  vec.y = 2;
  vec.z = 3.0;
  simulation_interface::toProto(vec, proto);
  EXPECT_DOUBLE_EQ(vec.x, proto.x());
  EXPECT_DOUBLE_EQ(vec.y, proto.y());
  EXPECT_DOUBLE_EQ(vec.z, proto.z());
  vec = geometry_msgs::msg::Vector3();
  EXPECT_DOUBLE_EQ(vec.x, 0);
  simulation_interface::toMsg(proto, vec);
  EXPECT_DOUBLE_EQ(vec.x, proto.x());
  EXPECT_DOUBLE_EQ(vec.y, proto.y());
  EXPECT_DOUBLE_EQ(vec.z, proto.z());
}

TEST(Conversion, ConvertTwist)
{
  geometry_msgs::Twist proto;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1;
  twist.linear.y = 2;
  twist.linear.z = 3.0;
  simulation_interface::toProto(twist, proto);
  EXPECT_DOUBLE_EQ(twist.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(twist.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(twist.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(twist.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(twist.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(twist.angular.z, proto.angular().z());
  twist = geometry_msgs::msg::Twist();
  EXPECT_DOUBLE_EQ(twist.linear.x, 0);
  simulation_interface::toMsg(proto, twist);
  EXPECT_DOUBLE_EQ(twist.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(twist.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(twist.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(twist.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(twist.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(twist.angular.z, proto.angular().z());
}

TEST(Conversion, ConvertAccel)
{
  geometry_msgs::Accel proto;
  geometry_msgs::msg::Accel accel;
  accel.linear.x = 1;
  accel.linear.y = 2;
  accel.linear.z = 3.0;
  simulation_interface::toProto(accel, proto);
  EXPECT_DOUBLE_EQ(accel.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(accel.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(accel.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(accel.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(accel.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(accel.angular.z, proto.angular().z());
  accel = geometry_msgs::msg::Accel();
  EXPECT_DOUBLE_EQ(accel.linear.x, 0);
  simulation_interface::toMsg(proto, accel);
  EXPECT_DOUBLE_EQ(accel.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(accel.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(accel.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(accel.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(accel.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(accel.angular.z, proto.angular().z());
}

TEST(Conversion, ConvertPerformance)
{
  openscenario_msgs::Performance proto;
  openscenario_msgs::msg::Performance performance;
  performance.max_speed = 10;
  performance.max_deceleration = 3;
  simulation_interface::toProto(performance, proto);
  EXPECT_DOUBLE_EQ(performance.max_acceleration, proto.max_acceleration());
  EXPECT_DOUBLE_EQ(performance.max_deceleration, proto.max_deceleration());
  EXPECT_DOUBLE_EQ(performance.max_speed, proto.max_speed());
  performance = openscenario_msgs::msg::Performance();
  EXPECT_DOUBLE_EQ(performance.max_speed, 0);
  simulation_interface::toMsg(proto, performance);
  EXPECT_DOUBLE_EQ(performance.max_acceleration, proto.max_acceleration());
  EXPECT_DOUBLE_EQ(performance.max_deceleration, proto.max_deceleration());
  EXPECT_DOUBLE_EQ(performance.max_speed, proto.max_speed());
}

TEST(Conversion, ConvertAxle)
{
  openscenario_msgs::Axle proto;
  openscenario_msgs::msg::Axle axle;
  axle.max_steering = 30;
  axle.position_x = 3;
  axle.position_z = 14;
  axle.track_width = -10;
  axle.wheel_diameter = 53;
  simulation_interface::toProto(axle, proto);
  EXPECT_DOUBLE_EQ(axle.max_steering, proto.max_steering());
  EXPECT_DOUBLE_EQ(axle.position_x, proto.position_x());
  EXPECT_DOUBLE_EQ(axle.position_z, proto.position_z());
  EXPECT_DOUBLE_EQ(axle.track_width, proto.track_width());
  EXPECT_DOUBLE_EQ(axle.wheel_diameter, proto.wheel_diameter());
  axle = openscenario_msgs::msg::Axle();
  EXPECT_DOUBLE_EQ(axle.max_steering, 0);
  simulation_interface::toMsg(proto, axle);
  EXPECT_DOUBLE_EQ(axle.max_steering, proto.max_steering());
  EXPECT_DOUBLE_EQ(axle.position_x, proto.position_x());
  EXPECT_DOUBLE_EQ(axle.position_z, proto.position_z());
  EXPECT_DOUBLE_EQ(axle.track_width, proto.track_width());
  EXPECT_DOUBLE_EQ(axle.wheel_diameter, proto.wheel_diameter());
}

TEST(Conversion, ConvertAxles)
{
  openscenario_msgs::Axles proto;
  openscenario_msgs::msg::Axles axles;
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
  EXPECT_DOUBLE_EQ(axles.front_axle.max_steering, proto.front_axle().max_steering());
  EXPECT_DOUBLE_EQ(axles.front_axle.position_x, proto.front_axle().position_x());
  EXPECT_DOUBLE_EQ(axles.front_axle.position_z, proto.front_axle().position_z());
  EXPECT_DOUBLE_EQ(axles.front_axle.track_width, proto.front_axle().track_width());
  EXPECT_DOUBLE_EQ(axles.front_axle.wheel_diameter, proto.front_axle().wheel_diameter());
  EXPECT_DOUBLE_EQ(axles.rear_axle.max_steering, proto.rear_axle().max_steering());
  EXPECT_DOUBLE_EQ(axles.rear_axle.position_x, proto.rear_axle().position_x());
  EXPECT_DOUBLE_EQ(axles.rear_axle.position_z, proto.rear_axle().position_z());
  EXPECT_DOUBLE_EQ(axles.rear_axle.track_width, proto.rear_axle().track_width());
  EXPECT_DOUBLE_EQ(axles.rear_axle.wheel_diameter, proto.rear_axle().wheel_diameter());
  axles = openscenario_msgs::msg::Axles();
  EXPECT_DOUBLE_EQ(axles.front_axle.max_steering, 0);
  simulation_interface::toMsg(proto, axles);
  EXPECT_DOUBLE_EQ(axles.front_axle.max_steering, proto.front_axle().max_steering());
  EXPECT_DOUBLE_EQ(axles.front_axle.position_x, proto.front_axle().position_x());
  EXPECT_DOUBLE_EQ(axles.front_axle.position_z, proto.front_axle().position_z());
  EXPECT_DOUBLE_EQ(axles.front_axle.track_width, proto.front_axle().track_width());
  EXPECT_DOUBLE_EQ(axles.front_axle.wheel_diameter, proto.front_axle().wheel_diameter());
  EXPECT_DOUBLE_EQ(axles.rear_axle.max_steering, proto.rear_axle().max_steering());
  EXPECT_DOUBLE_EQ(axles.rear_axle.position_x, proto.rear_axle().position_x());
  EXPECT_DOUBLE_EQ(axles.rear_axle.position_z, proto.rear_axle().position_z());
  EXPECT_DOUBLE_EQ(axles.rear_axle.track_width, proto.rear_axle().track_width());
  EXPECT_DOUBLE_EQ(axles.rear_axle.wheel_diameter, proto.rear_axle().wheel_diameter());
}

/*
TEST(Conversion, ConvertProperty)
{
  openscenario_msgs::Property proto;
  openscenario_msgs::msg::Property p;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  // p.is_ego = true;
  // EXPECT_EQ(proto.is_ego(), p.is_ego);
  // p.is_ego = false;
  // EXPECT_EQ(proto.is_ego(), p.is_ego);
}
*/

TEST(Conversion, ConvertVehicleParametrs)
{
  openscenario_msgs::VehicleParameters proto;
  openscenario_msgs::msg::VehicleParameters p;
  // p.property.is_ego = true;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  // EXPECT_EQ(proto.property().is_ego(), p.property.is_ego);
  // p.property.is_ego = false;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  // EXPECT_EQ(proto.property().is_ego(), p.property.is_ego);
}

TEST(Conversion, ConvertActionStatus)
{
  openscenario_msgs::ActionStatus proto;
  openscenario_msgs::msg::ActionStatus action;
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
  EXPECT_STREQ(action.current_action.c_str(), proto.current_action().c_str());
  EXPECT_DOUBLE_EQ(action.twist.linear.x, proto.twist().linear().x());
  EXPECT_DOUBLE_EQ(action.twist.linear.y, proto.twist().linear().y());
  EXPECT_DOUBLE_EQ(action.twist.linear.z, proto.twist().linear().z());
  EXPECT_DOUBLE_EQ(action.twist.angular.x, proto.twist().angular().x());
  EXPECT_DOUBLE_EQ(action.twist.angular.y, proto.twist().angular().y());
  EXPECT_DOUBLE_EQ(action.twist.angular.z, proto.twist().angular().z());
  EXPECT_DOUBLE_EQ(action.accel.linear.x, proto.accel().linear().x());
  EXPECT_DOUBLE_EQ(action.accel.linear.y, proto.accel().linear().y());
  EXPECT_DOUBLE_EQ(action.accel.linear.z, proto.accel().linear().z());
  EXPECT_DOUBLE_EQ(action.accel.angular.x, proto.accel().angular().x());
  EXPECT_DOUBLE_EQ(action.accel.angular.y, proto.accel().angular().y());
  EXPECT_DOUBLE_EQ(action.accel.angular.z, proto.accel().angular().z());
}

TEST(Conversion, Time)
{
  builtin_interfaces::Time proto;
  builtin_interfaces::msg::Time msg;
  msg.nanosec = 1;
  msg.sec = 2;
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.nanosec, proto.nanosec());
  EXPECT_EQ(msg.sec, proto.sec());
  msg.nanosec = 0;
  msg.sec = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.nanosec, proto.nanosec());
  EXPECT_EQ(msg.sec, proto.sec());
}

TEST(Conversion, Duration)
{
  builtin_interfaces::Duration proto;
  builtin_interfaces::msg::Duration msg;
  msg.nanosec = 1;
  msg.sec = 2;
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.nanosec, proto.nanosec());
  EXPECT_EQ(msg.sec, proto.sec());
  msg.nanosec = 0;
  msg.sec = 0;
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.nanosec, proto.nanosec());
  EXPECT_EQ(msg.sec, proto.sec());
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

TEST(Conversion, ControlCommand)
{
  autoware_control_msgs::ControlCommand proto;
  autoware_control_msgs::msg::ControlCommand msg;
  msg.acceleration = 3;
  msg.steering_angle = 1.4;
  msg.steering_angle_velocity = 13.4;
  msg.velocity = 11.3;
  simulation_interface::toProto(msg, proto);
  EXPECT_CONTROL_COMMAND_EQ(msg, proto);
  msg.acceleration = 0;
  msg.steering_angle = 0;
  msg.steering_angle_velocity = 0;
  msg.velocity = 0;
  simulation_interface::toMsg(proto, msg);
}

TEST(Conversion, Shift)
{
  autoware_vehicle_msgs::Shift proto;
  proto.set_data(autoware_vehicle_msgs::SHIFT_POSITIONS::PARKING);
  autoware_vehicle_msgs::msg::Shift msg;
  msg.data = autoware_vehicle_msgs::msg::Shift::LOW;
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto), common::scenario_simulator_exception::SemanticError);
}

TEST(Conversion, VehicleCommand)
{
  autoware_vehicle_msgs::VehicleCommand proto;
  autoware_vehicle_msgs::msg::VehicleCommand msg;
  msg.control.velocity = 1.2;
  msg.control.steering_angle_velocity = 19.3;
  msg.control.steering_angle = 12.0;
  msg.control.steering_angle_velocity = 192.4;
  msg.shift.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
  msg.emergency = 1;
  msg.header.frame_id = "base_link";
  msg.header.stamp.nanosec = 99;
  msg.header.stamp.sec = 3;
  simulation_interface::toProto(msg, proto);
  EXPECT_CONTROL_COMMAND_EQ(msg.control, proto.control());
  EXPECT_EQ(msg.shift.data, proto.shift().data());
  EXPECT_TRUE(msg.shift.data == proto.shift().data());
  EXPECT_EQ(msg.shift.data, proto.shift().data());
  msg.shift.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto), common::scenario_simulator_exception::SemanticError);
  EXPECT_HEADER_EQ(msg.header, proto.header());
  EXPECT_EQ(msg.emergency, proto.emergency());
  msg = autoware_vehicle_msgs::msg::VehicleCommand();
  simulation_interface::toMsg(proto, msg);
  EXPECT_CONTROL_COMMAND_EQ(msg.control, proto.control());
  EXPECT_EQ(msg.shift.data, proto.shift().data());
  EXPECT_TRUE(msg.shift.data == proto.shift().data());
  EXPECT_EQ(msg.shift.data, proto.shift().data());
  msg.shift.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto), common::scenario_simulator_exception::SemanticError);
  EXPECT_HEADER_EQ(msg.header, proto.header());
  EXPECT_EQ(msg.emergency, proto.emergency());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
