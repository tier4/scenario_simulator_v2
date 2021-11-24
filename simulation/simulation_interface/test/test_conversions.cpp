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
  EXPECT_DOUBLE_EQ(performance.max_speed, 0);
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
  p.vehicle_category = "bar";
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
  p.pedestrian_category = "bar";
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
  simulation_interface::toMsg(proto, status);
  EXPECT_ENTITY_STATUS_EQ(status, proto);
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
  EXPECT_CONTROL_COMMAND_EQ(msg, proto);
}

TEST(Conversion, Shift)
{
  /**
   * @note Convert low shift value
   */
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
  /**
   * @note Convert parking shift value
   */
  msg.data = autoware_vehicle_msgs::msg::Shift::PARKING;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = autoware_vehicle_msgs::msg::Shift::LOW;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.data, proto.data());
  /**
   * @note Convert drive shift value
   */
  msg.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = autoware_vehicle_msgs::msg::Shift::LOW;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.data, proto.data());
  /**
   * @note Convert reverse shift value
   */
  msg.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = autoware_vehicle_msgs::msg::Shift::LOW;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.data, proto.data());
  /**
   * @note Convert none shift value
   */
  msg.data = autoware_vehicle_msgs::msg::Shift::NONE;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toProto(msg, proto);
  EXPECT_EQ(msg.data, proto.data());
  msg.data = autoware_vehicle_msgs::msg::Shift::LOW;
  EXPECT_FALSE(msg.data == proto.data());
  simulation_interface::toMsg(proto, msg);
  EXPECT_EQ(msg.data, proto.data());
  /**
   * @note Invalid value input
   */
  msg.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto),
    common::scenario_simulator_exception::SimulationError);
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
  EXPECT_VEHICLE_COMMAND_EQ(msg, proto);
  msg.shift.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto),
    common::scenario_simulator_exception::SimulationError);
  EXPECT_HEADER_EQ(msg.header, proto.header());
  EXPECT_EQ(msg.emergency, proto.emergency());
  msg = autoware_vehicle_msgs::msg::VehicleCommand();
  simulation_interface::toMsg(proto, msg);
  EXPECT_VEHICLE_COMMAND_EQ(msg, proto);
  msg.shift.data = 1023;
  EXPECT_THROW(
    simulation_interface::toProto(msg, proto),
    common::scenario_simulator_exception::SimulationError);
  EXPECT_HEADER_EQ(msg.header, proto.header());
  EXPECT_EQ(msg.emergency, proto.emergency());
}

TEST(Conversion, EntityType)
{
  traffic_simulator_msgs::EntityType proto;
  traffic_simulator_msgs::msg::EntityType msg;
  msg.type = msg.VEHICLE;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto, traffic_simulator_msgs::EntityType::VEHICLE);
  msg.type = msg.MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto, traffic_simulator_msgs::EntityType::MISC_OBJECT);
  proto = traffic_simulator_msgs::EntityType::VEHICLE;
  msg.type = msg.EGO;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::VEHICLE);
  msg.type = msg.VEHICLE;
  proto = traffic_simulator_msgs::EntityType::EGO;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::EGO);
  msg.type = msg.VEHICLE;
  proto = traffic_simulator_msgs::EntityType::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
  proto = traffic_simulator_msgs::EntityType::MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toMsg(proto, msg));
  EXPECT_EQ(msg.type, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT);
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

TEST(Conversion, TrafficLights)
{
  simulation_api_schema::TrafficLightState proto;
  autoware_auto_perception_msgs::msg::TrafficSignal msg;
  msg.map_primitive_id = 123;
  autoware_auto_perception_msgs::msg::TrafficLight ls0;
  autoware_auto_perception_msgs::msg::TrafficLight ls1;
  autoware_auto_perception_msgs::msg::TrafficLight ls2;
  autoware_auto_perception_msgs::msg::TrafficLight ls3;
  autoware_auto_perception_msgs::msg::TrafficLight ls4;
  autoware_auto_perception_msgs::msg::TrafficLight ls5;
  autoware_auto_perception_msgs::msg::TrafficLight ls6;
  autoware_auto_perception_msgs::msg::TrafficLight ls7;
  ls0.color = ls0.UNKNOWN;
  ls0.confidence = 12.12;

  ls1.color = ls1.RED;
  ls2.color = ls2.GREEN;
  ls3.color = ls3.AMBER;

  ls4.shape = ls4.LEFT_ARROW;
  ls4.color = ls4.RED;
  ls5.shape = ls5.RIGHT_ARROW;
  ls5.color = ls5.RED;
  ls6.shape = ls6.UP_ARROW;
  ls6.color = ls6.RED;
  ls7.shape = ls7.DOWN_ARROW;
  ls7.color = ls7.RED;

  msg.lights = {ls0, ls1, ls2, ls3, ls4, ls5, ls6, ls7};

  EXPECT_NO_THROW(simulation_interface::toProto(msg, proto));
  EXPECT_EQ(proto.id(), 123);
  EXPECT_NE(proto.lights()[0].confidence(), 12.12);
  EXPECT_EQ(proto.lights()[0].type(), simulation_api_schema::TrafficLightState::LampState::UNKNOWN);
  EXPECT_EQ(proto.lights()[1].type(), simulation_api_schema::TrafficLightState::LampState::RED);
  EXPECT_EQ(proto.lights()[2].type(), simulation_api_schema::TrafficLightState::LampState::GREEN);
  EXPECT_EQ(proto.lights()[3].type(), simulation_api_schema::TrafficLightState::LampState::YELLOW);
  EXPECT_EQ(proto.lights()[4].type(), simulation_api_schema::TrafficLightState::LampState::LEFT);
  EXPECT_EQ(proto.lights()[5].type(), simulation_api_schema::TrafficLightState::LampState::RIGHT);
  EXPECT_EQ(proto.lights()[6].type(), simulation_api_schema::TrafficLightState::LampState::UP);
  EXPECT_EQ(proto.lights()[7].type(), simulation_api_schema::TrafficLightState::LampState::DOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
