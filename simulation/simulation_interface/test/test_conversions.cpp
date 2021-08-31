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

#include "expect_equal_macros.hpp"

void checkActionStatus(
  openscenario_msgs::ActionStatus proto, openscenario_msgs::msg::ActionStatus action)
{
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
  openscenario_msgs::Performance proto;
  openscenario_msgs::msg::Performance performance;
  performance.max_speed = 10;
  performance.max_deceleration = 3;
  simulation_interface::toProto(performance, proto);
  EXPECT_PERFORMANCE_EQ(performance, proto);
  EXPECT_DOUBLE_EQ(performance.max_acceleration, proto.max_acceleration());
  EXPECT_DOUBLE_EQ(performance.max_deceleration, proto.max_deceleration());
  EXPECT_DOUBLE_EQ(performance.max_speed, proto.max_speed());
  performance = openscenario_msgs::msg::Performance();
  EXPECT_DOUBLE_EQ(performance.max_speed, 0);
  simulation_interface::toMsg(proto, performance);
  EXPECT_PERFORMANCE_EQ(performance, proto);
}

TEST(Conversion, Axle)
{
  openscenario_msgs::Axle proto;
  openscenario_msgs::msg::Axle axle;
  axle.max_steering = 30;
  axle.position_x = 3;
  axle.position_z = 14;
  axle.track_width = -10;
  axle.wheel_diameter = 53;
  simulation_interface::toProto(axle, proto);
  EXPECT_AXLE_EQ(axle, proto);
  axle = openscenario_msgs::msg::Axle();
  EXPECT_DOUBLE_EQ(axle.max_steering, 0);
  simulation_interface::toMsg(proto, axle);
  EXPECT_AXLE_EQ(axle, proto);
}

TEST(Conversion, Axles)
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
  EXPECT_AXLES_EQ(axles, proto);
  axles = openscenario_msgs::msg::Axles();
  EXPECT_DOUBLE_EQ(axles.front_axle.max_steering, 0);
  simulation_interface::toMsg(proto, axles);
  EXPECT_AXLES_EQ(axles, proto);
}

TEST(Conversion, BoundingBox)
{
  openscenario_msgs::BoundingBox proto;
  openscenario_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  simulation_interface::toProto(box, proto);
  EXPECT_BOUNDING_BOX_EQ(box, proto);
  box = openscenario_msgs::msg::BoundingBox();
  EXPECT_DOUBLE_EQ(box.center.x, 0);
  simulation_interface::toMsg(proto, box);
  EXPECT_BOUNDING_BOX_EQ(box, proto);
}

TEST(Conversion, VehicleParametrs)
{
  openscenario_msgs::VehicleParameters proto;
  openscenario_msgs::msg::VehicleParameters p;
  p.name = "foo";
  p.vehicle_category = "bar";
  openscenario_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  openscenario_msgs::msg::Performance performance;
  performance.max_speed = 10;
  performance.max_deceleration = 3;
  p.performance = performance;
  openscenario_msgs::msg::Axle axle;
  axle.max_steering = 30;
  axle.position_x = 3;
  axle.position_z = 14;
  axle.track_width = -10;
  axle.wheel_diameter = 53;
  p.axles.front_axle = axle;
  p.axles.rear_axle = axle;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_VEHICLE_PARAMETERS_EQ(p, proto);
  p = openscenario_msgs::msg::VehicleParameters();
  EXPECT_DOUBLE_EQ(p.bounding_box.dimensions.x, 0);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_VEHICLE_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, PedestrianParametrs)
{
  openscenario_msgs::PedestrianParameters proto;
  openscenario_msgs::msg::PedestrianParameters p;
  p.name = "foo";
  p.pedestrian_category = "bar";
  openscenario_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(p, proto);
  p = openscenario_msgs::msg::PedestrianParameters();
  EXPECT_DOUBLE_EQ(p.bounding_box.dimensions.x, 0);
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, MiscObjectParametrs)
{
  openscenario_msgs::MiscObjectParameters proto;
  openscenario_msgs::msg::MiscObjectParameters p;
  openscenario_msgs::msg::BoundingBox box;
  box.center.x = 1.0;
  box.center.y = 1.23;
  box.center.z = 43.0;
  box.dimensions.x = 12.3;
  box.dimensions.y = 3.9;
  box.dimensions.z = 4.0;
  p.bounding_box = box;
  p.misc_object_category = "obstacle";
  EXPECT_NO_THROW(simulation_interface::toProto(p, proto));
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(p, proto);
  p.misc_object_category = "";
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, p));
  EXPECT_STREQ(p.misc_object_category.c_str(), proto.misc_object_category().c_str());
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(p, proto);
}

TEST(Conversion, ActionStatus)
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
  checkActionStatus(proto, action);
  action = openscenario_msgs::msg::ActionStatus();
  simulation_interface::toMsg(proto, action);
  checkActionStatus(proto, action);
}

TEST(Conversion, EntityStatus)
{
  openscenario_msgs::EntityStatus proto;
  openscenario_msgs::msg::EntityStatus status;
  status.name = "test";
  // status.pose = "example";
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

TEST(Conversion, EntityType)
{
  openscenario_msgs::EntityType proto;
  openscenario_msgs::msg::EntityType msg;
  msg.type = msg.VEHICLE;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto, openscenario_msgs::EntityType::VEHICLE);
  msg.type = msg.MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto, openscenario_msgs::EntityType::MISC_OBJECT);
  proto = openscenario_msgs::EntityType::VEHICLE;
  msg.type = msg.EGO;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, openscenario_msgs::msg::EntityType::VEHICLE);
  msg.type = msg.VEHICLE;
  proto = openscenario_msgs::EntityType::EGO;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, openscenario_msgs::msg::EntityType::EGO);
  msg.type = msg.VEHICLE;
  proto = openscenario_msgs::EntityType::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, openscenario_msgs::msg::EntityType::PEDESTRIAN);
  proto = openscenario_msgs::EntityType::MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, openscenario_msgs::msg::EntityType::MISC_OBJECT);
}

TEST(Conversion, LaneletPose)
{
  openscenario_msgs::msg::LaneletPose pose;
  openscenario_msgs::LaneletPose proto;
  pose.lanelet_id = 23;
  pose.s = 1.0;
  pose.offset = 3.5;
  pose.rpy.x = 3.4;
  pose.rpy.y = 5.1;
  pose.rpy.z = 1.3;
  EXPECT_NO_THROW(simulation_interface::toProto(pose, proto));
  EXPECT_EQ(pose.lanelet_id, proto.lanelet_id());
  EXPECT_DOUBLE_EQ(pose.s, proto.s());
  EXPECT_DOUBLE_EQ(pose.offset, proto.offset());
  EXPECT_DOUBLE_EQ(pose.rpy.x, proto.rpy().x());
  EXPECT_DOUBLE_EQ(pose.rpy.y, proto.rpy().y());
  EXPECT_DOUBLE_EQ(pose.rpy.z, proto.rpy().z());
  pose = openscenario_msgs::msg::LaneletPose();
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, pose));
  EXPECT_EQ(pose.lanelet_id, proto.lanelet_id());
  EXPECT_DOUBLE_EQ(pose.s, proto.s());
  EXPECT_DOUBLE_EQ(pose.offset, proto.offset());
  EXPECT_DOUBLE_EQ(pose.rpy.x, proto.rpy().x());
  EXPECT_DOUBLE_EQ(pose.rpy.y, proto.rpy().y());
  EXPECT_DOUBLE_EQ(pose.rpy.z, proto.rpy().z());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
