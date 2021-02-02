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

#include <gtest/gtest.h>

#include <geometry_msgs.pb.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <xmlrpc_interface/conversions.hpp>

#include <string>

TEST(Conversion, ConvertInitializeResponse)
{
  simulation_api_schema::InitializeResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("test");
  XmlRpc::XmlRpcValue xml;
  EXPECT_NO_THROW(xmlrpc_interface::fromProto(res, xml));
  std::string description = xml[xmlrpc_interface::key::description];
  EXPECT_STREQ(description.c_str(), "test");
  res.mutable_result()->set_description("");
  EXPECT_NO_THROW(xmlrpc_interface::toProto(xml, res));
  EXPECT_STREQ(res.result().description().c_str(), "test");
  std::string serialized_str = "";
  res.SerializeToString(&serialized_str);
}

TEST(Conversion, ConvertInitializeRequest)
{
  simulation_api_schema::InitializeRequest req;
  req.set_realtime_factor(0.1);
  req.set_step_time(0.5);
  XmlRpc::XmlRpcValue xml;
  xmlrpc_interface::fromProto(req, xml);
  EXPECT_DOUBLE_EQ(req.step_time(), xml[xmlrpc_interface::key::step_time]);
  EXPECT_DOUBLE_EQ(req.realtime_factor(), xml[xmlrpc_interface::key::realtime_factor]);
  req.set_realtime_factor(0);
  req.set_step_time(0);
  xmlrpc_interface::toProto(xml, req);
  EXPECT_DOUBLE_EQ(req.step_time(), xml[xmlrpc_interface::key::step_time]);
  EXPECT_DOUBLE_EQ(req.realtime_factor(), xml[xmlrpc_interface::key::realtime_factor]);
}

TEST(Conversion, ConvertPoint)
{
  geometry_msgs::Point proto;
  geometry_msgs::msg::Point p;
  p.x = 1.0;
  p.y = 2;
  p.z = 3.1;
  xmlrpc_interface::toProto(p, proto);
  EXPECT_DOUBLE_EQ(p.x, proto.x());
  EXPECT_DOUBLE_EQ(p.y, proto.y());
  EXPECT_DOUBLE_EQ(p.z, proto.z());
  p = geometry_msgs::msg::Point();
  EXPECT_DOUBLE_EQ(p.x, 0);
  xmlrpc_interface::toMsg(proto, p);
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
  xmlrpc_interface::toProto(q, proto);
  EXPECT_DOUBLE_EQ(q.x, proto.x());
  EXPECT_DOUBLE_EQ(q.y, proto.y());
  EXPECT_DOUBLE_EQ(q.z, proto.z());
  EXPECT_DOUBLE_EQ(q.w, proto.w());
  q = geometry_msgs::msg::Quaternion();
  EXPECT_DOUBLE_EQ(q.x, 0);
  xmlrpc_interface::toMsg(proto, q);
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
  xmlrpc_interface::toProto(p, proto);
  EXPECT_DOUBLE_EQ(p.position.x, proto.position().x());
  EXPECT_DOUBLE_EQ(p.position.y, proto.position().y());
  EXPECT_DOUBLE_EQ(p.position.z, proto.position().z());
  EXPECT_DOUBLE_EQ(p.orientation.x, proto.orientation().x());
  EXPECT_DOUBLE_EQ(p.orientation.y, proto.orientation().y());
  EXPECT_DOUBLE_EQ(p.orientation.z, proto.orientation().z());
  EXPECT_DOUBLE_EQ(p.orientation.w, proto.orientation().w());
  p = geometry_msgs::msg::Pose();
  EXPECT_DOUBLE_EQ(p.position.x, 0);
  xmlrpc_interface::toMsg(proto, p);
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
  xmlrpc_interface::toProto(vec, proto);
  EXPECT_DOUBLE_EQ(vec.x, proto.x());
  EXPECT_DOUBLE_EQ(vec.y, proto.y());
  EXPECT_DOUBLE_EQ(vec.z, proto.z());
  vec = geometry_msgs::msg::Vector3();
  EXPECT_DOUBLE_EQ(vec.x, 0);
  xmlrpc_interface::toMsg(proto, vec);
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
  xmlrpc_interface::toProto(twist, proto);
  EXPECT_DOUBLE_EQ(twist.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(twist.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(twist.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(twist.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(twist.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(twist.angular.z, proto.angular().z());
  twist = geometry_msgs::msg::Twist();
  EXPECT_DOUBLE_EQ(twist.linear.x, 0);
  xmlrpc_interface::toMsg(proto, twist);
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
  xmlrpc_interface::toProto(accel, proto);
  EXPECT_DOUBLE_EQ(accel.linear.x, proto.linear().x());
  EXPECT_DOUBLE_EQ(accel.linear.y, proto.linear().y());
  EXPECT_DOUBLE_EQ(accel.linear.z, proto.linear().z());
  EXPECT_DOUBLE_EQ(accel.angular.x, proto.angular().x());
  EXPECT_DOUBLE_EQ(accel.angular.y, proto.angular().y());
  EXPECT_DOUBLE_EQ(accel.angular.z, proto.angular().z());
  accel = geometry_msgs::msg::Accel();
  EXPECT_DOUBLE_EQ(accel.linear.x, 0);
  xmlrpc_interface::toMsg(proto, accel);
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
  xmlrpc_interface::toProto(performance, proto);
  EXPECT_DOUBLE_EQ(performance.max_acceleration, proto.max_acceleration());
  EXPECT_DOUBLE_EQ(performance.max_deceleration, proto.max_deceleration());
  EXPECT_DOUBLE_EQ(performance.max_speed, proto.max_speed());
  performance = openscenario_msgs::msg::Performance();
  EXPECT_DOUBLE_EQ(performance.max_speed, 0);
  xmlrpc_interface::toMsg(proto, performance);
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
  xmlrpc_interface::toProto(axle, proto);
  EXPECT_DOUBLE_EQ(axle.max_steering, proto.max_steering());
  EXPECT_DOUBLE_EQ(axle.position_x, proto.position_x());
  EXPECT_DOUBLE_EQ(axle.position_z, proto.position_z());
  EXPECT_DOUBLE_EQ(axle.track_width, proto.track_width());
  EXPECT_DOUBLE_EQ(axle.wheel_diameter, proto.wheel_diameter());
  axle = openscenario_msgs::msg::Axle();
  EXPECT_DOUBLE_EQ(axle.max_steering, 0);
  xmlrpc_interface::toMsg(proto, axle);
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
  xmlrpc_interface::toProto(axles, proto);
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
  xmlrpc_interface::toMsg(proto, axles);
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
