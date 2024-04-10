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

#include "../include/expect_equal_macros.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Conversion, toProto_Point)
{
  geometry_msgs::msg::Point src_msg;
  geometry_msgs::Point dst_proto;
  src_msg.y = 3.0;
  src_msg.z = 5.0;
  src_msg.z = 7.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_POINT_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Point)
{
  geometry_msgs::msg::Point init_msg;
  geometry_msgs::Point src_proto;
  geometry_msgs::msg::Point dst_msg;
  init_msg.y = 3.0;
  init_msg.z = 5.0;
  init_msg.z = 7.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_POINT_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Quaternion)
{
  geometry_msgs::msg::Quaternion src_msg;
  geometry_msgs::Quaternion dst_proto;
  src_msg.x = 3.0;
  src_msg.y = 5.0;
  src_msg.z = 7.0;
  src_msg.w = 11.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_QUATERNION_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Quaternion)
{
  geometry_msgs::msg::Quaternion init_msg;
  geometry_msgs::Quaternion src_proto;
  geometry_msgs::msg::Quaternion dst_msg;
  init_msg.x = 3.0;
  init_msg.y = 5.0;
  init_msg.z = 7.0;
  init_msg.w = 11.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_QUATERNION_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Pose)
{
  geometry_msgs::msg::Pose src_msg;
  geometry_msgs::Pose dst_proto;
  src_msg.position.x = 3.0;
  src_msg.position.y = 5.0;
  src_msg.position.z = 7.0;
  src_msg.orientation.x = 11.0;
  src_msg.orientation.y = 13.0;
  src_msg.orientation.z = 17.0;
  src_msg.orientation.w = 19.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_POSE_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Pose)
{
  geometry_msgs::msg::Pose init_msg;
  geometry_msgs::Pose src_proto;
  geometry_msgs::msg::Pose dst_msg;
  init_msg.position.x = 3.0;
  init_msg.position.y = 5.0;
  init_msg.position.z = 7.0;
  init_msg.orientation.x = 11.0;
  init_msg.orientation.y = 13.0;
  init_msg.orientation.z = 17.0;
  init_msg.orientation.w = 19.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_POSE_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Vector)
{
  geometry_msgs::msg::Vector3 src_msg;
  geometry_msgs::Vector3 dst_proto;
  src_msg.x = 3.0;
  src_msg.y = 5.0;
  src_msg.z = 7.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_VECTOR3_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Vector)
{
  geometry_msgs::msg::Vector3 init_msg;
  geometry_msgs::Vector3 src_proto;
  geometry_msgs::msg::Vector3 dst_msg;
  init_msg.x = 3.0;
  init_msg.y = 5.0;
  init_msg.z = 7.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_VECTOR3_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Twist)
{
  geometry_msgs::msg::Twist src_msg;
  geometry_msgs::Twist dst_proto;
  src_msg.linear.x = 3.0;
  src_msg.linear.y = 5.0;
  src_msg.linear.z = 7.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_TWIST_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Twist)
{
  geometry_msgs::msg::Twist init_msg;
  geometry_msgs::Twist src_proto;
  geometry_msgs::msg::Twist dst_msg;
  init_msg.linear.x = 3.0;
  init_msg.linear.y = 5.0;
  init_msg.linear.z = 7.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_TWIST_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Accel)
{
  geometry_msgs::msg::Accel src_msg;
  geometry_msgs::Accel dst_proto;
  src_msg.linear.x = 3.0;
  src_msg.linear.y = 5.0;
  src_msg.linear.z = 7.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_ACCEL_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Accel)
{
  geometry_msgs::msg::Accel init_msg;
  geometry_msgs::Accel src_proto;
  geometry_msgs::msg::Accel dst_msg;
  init_msg.linear.x = 3.0;
  init_msg.linear.y = 5.0;
  init_msg.linear.z = 7.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_ACCEL_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Performance)
{
  traffic_simulator_msgs::msg::Performance src_msg;
  traffic_simulator_msgs::Performance dst_proto;
  src_msg.max_speed = 3.0;
  src_msg.max_deceleration = 5.0;
  src_msg.max_acceleration = 7.0;
  src_msg.max_deceleration_rate = 11.0;
  src_msg.max_acceleration_rate = 13.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_PERFORMANCE_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Performance)
{
  traffic_simulator_msgs::msg::Performance init_msg;
  traffic_simulator_msgs::Performance src_proto;
  traffic_simulator_msgs::msg::Performance dst_msg;
  init_msg.max_speed = 3.0;
  init_msg.max_deceleration = 5.0;
  init_msg.max_acceleration = 7.0;
  init_msg.max_deceleration_rate = 11.0;
  init_msg.max_acceleration_rate = 13.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_PERFORMANCE_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Axle)
{
  traffic_simulator_msgs::msg::Axle src_msg;
  traffic_simulator_msgs::Axle dst_proto;
  src_msg.max_steering = 3.0;
  src_msg.position_x = 5.0;
  src_msg.position_z = 7.0;
  src_msg.track_width = 11.0;
  src_msg.wheel_diameter = 13.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_AXLE_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Axle)
{
  traffic_simulator_msgs::msg::Axle init_msg;
  traffic_simulator_msgs::Axle src_proto;
  traffic_simulator_msgs::msg::Axle dst_msg;
  init_msg.max_steering = 3.0;
  init_msg.position_x = 5.0;
  init_msg.position_z = 7.0;
  init_msg.track_width = 11.0;
  init_msg.wheel_diameter = 13.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_AXLE_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Axles)
{
  traffic_simulator_msgs::msg::Axles src_msg;
  traffic_simulator_msgs::Axles dst_proto;
  src_msg.front_axle.max_steering = 3.0;
  src_msg.front_axle.position_x = 5.0;
  src_msg.front_axle.position_z = 7.0;
  src_msg.front_axle.track_width = 11.0;
  src_msg.front_axle.wheel_diameter = 13.0;
  src_msg.rear_axle.max_steering = 17.0;
  src_msg.rear_axle.position_x = 19.0;
  src_msg.rear_axle.position_z = 23.0;
  src_msg.rear_axle.track_width = 29.0;
  src_msg.rear_axle.wheel_diameter = 31.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_AXLES_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Axles)
{
  traffic_simulator_msgs::msg::Axles init_msg;
  traffic_simulator_msgs::Axles src_proto;
  traffic_simulator_msgs::msg::Axles dst_msg;
  init_msg.front_axle.max_steering = 3.0;
  init_msg.front_axle.position_x = 5.0;
  init_msg.front_axle.position_z = 7.0;
  init_msg.front_axle.track_width = 11.0;
  init_msg.front_axle.wheel_diameter = 13.0;
  init_msg.rear_axle.max_steering = 17.0;
  init_msg.rear_axle.position_x = 19.0;
  init_msg.rear_axle.position_z = 23.0;
  init_msg.rear_axle.track_width = 29.0;
  init_msg.rear_axle.wheel_diameter = 31.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_AXLES_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_BoundingBox)
{
  traffic_simulator_msgs::msg::BoundingBox src_msg;
  traffic_simulator_msgs::BoundingBox dst_proto;
  src_msg.center.x = 3.0;
  src_msg.center.y = 5.0;
  src_msg.center.z = 7.0;
  src_msg.dimensions.x = 11.0;
  src_msg.dimensions.y = 13.0;
  src_msg.dimensions.z = 17.0;
  simulation_interface::toProto(src_msg, dst_proto);
  EXPECT_BOUNDING_BOX_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_BoundingBox)
{
  traffic_simulator_msgs::msg::BoundingBox init_msg;
  traffic_simulator_msgs::BoundingBox src_proto;
  traffic_simulator_msgs::msg::BoundingBox dst_msg;
  init_msg.center.x = 3.0;
  init_msg.center.y = 5.0;
  init_msg.center.z = 7.0;
  init_msg.dimensions.x = 11.0;
  init_msg.dimensions.y = 13.0;
  init_msg.dimensions.z = 17.0;
  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);
  EXPECT_BOUNDING_BOX_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_VehicleParameters)
{
  traffic_simulator_msgs::msg::VehicleParameters src_msg;
  traffic_simulator_msgs::VehicleParameters dst_proto;
  src_msg.name = "test";
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  src_msg.bounding_box = box_msg;
  traffic_simulator_msgs::msg::Performance performance_msg;
  performance_msg.max_speed = 19.0;
  performance_msg.max_deceleration = 23.0;
  src_msg.performance = performance_msg;
  traffic_simulator_msgs::msg::Axle axle_msg;
  axle_msg.max_steering = 29.0;
  axle_msg.position_x = 31.0;
  axle_msg.position_z = 37.0;
  axle_msg.track_width = 41.0;
  axle_msg.wheel_diameter = 43.0;
  src_msg.axles.front_axle = axle_msg;
  src_msg.axles.rear_axle = axle_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_VEHICLE_PARAMETERS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_VehicleParameters)
{
  traffic_simulator_msgs::msg::VehicleParameters init_msg;
  traffic_simulator_msgs::VehicleParameters src_proto;
  traffic_simulator_msgs::msg::VehicleParameters dst_msg;
  init_msg.name = "test";
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  init_msg.bounding_box = box_msg;
  traffic_simulator_msgs::msg::Performance performance_msg;
  performance_msg.max_speed = 19.0;
  performance_msg.max_deceleration = 23.0;
  init_msg.performance = performance_msg;
  traffic_simulator_msgs::msg::Axle axle_msg;
  axle_msg.max_steering = 29.0;
  axle_msg.position_x = 31.0;
  axle_msg.position_z = 37.0;
  axle_msg.track_width = 41.0;
  axle_msg.wheel_diameter = 43.0;
  init_msg.axles.front_axle = axle_msg;
  init_msg.axles.rear_axle = axle_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_VEHICLE_PARAMETERS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_PedestrianParameters)
{
  traffic_simulator_msgs::msg::PedestrianParameters src_msg;
  traffic_simulator_msgs::PedestrianParameters dst_proto;
  src_msg.name = "test";
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  src_msg.bounding_box = box_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_PedestrianParameters)
{
  traffic_simulator_msgs::msg::PedestrianParameters init_msg;
  traffic_simulator_msgs::PedestrianParameters src_proto;
  traffic_simulator_msgs::msg::PedestrianParameters dst_msg;
  init_msg.name = "test";
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  init_msg.bounding_box = box_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_PEDESTRIAN_PARAMETERS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_MiscObjectParameters)
{
  traffic_simulator_msgs::msg::MiscObjectParameters src_msg;
  traffic_simulator_msgs::MiscObjectParameters dst_proto;
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  src_msg.bounding_box = box_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_MiscObjectParameters)
{
  traffic_simulator_msgs::msg::MiscObjectParameters init_msg;
  traffic_simulator_msgs::MiscObjectParameters src_proto;
  traffic_simulator_msgs::msg::MiscObjectParameters dst_msg;
  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 3.0;
  box_msg.center.y = 5.0;
  box_msg.center.z = 7.0;
  box_msg.dimensions.x = 11.0;
  box_msg.dimensions.y = 13.0;
  box_msg.dimensions.z = 17.0;
  init_msg.bounding_box = box_msg;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_MISC_OBJECT_PARAMETERS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_ActionStatus)
{
  traffic_simulator_msgs::msg::ActionStatus src_msg;
  traffic_simulator_msgs::ActionStatus dst_proto;

  src_msg.current_action = "test";
  src_msg.twist.linear.x = 3.0;
  src_msg.twist.linear.y = 5.0;
  src_msg.twist.linear.z = 7.0;
  src_msg.twist.angular.x = 11.0;
  src_msg.twist.angular.y = 13.0;
  src_msg.twist.angular.z = 17.0;

  src_msg.accel.linear.x = 19.0;
  src_msg.accel.linear.y = 23.0;
  src_msg.accel.linear.z = 29.0;
  src_msg.accel.angular.x = 31.0;
  src_msg.accel.angular.y = 37.0;
  src_msg.accel.angular.z = 41.0;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_ACTION_STATUS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_ActionStatus)
{
  traffic_simulator_msgs::msg::ActionStatus init_msg;
  traffic_simulator_msgs::ActionStatus src_proto;
  traffic_simulator_msgs::msg::ActionStatus dst_msg;

  init_msg.current_action = "test";
  init_msg.twist.linear.x = 3.0;
  init_msg.twist.linear.y = 5.0;
  init_msg.twist.linear.z = 7.0;
  init_msg.twist.angular.x = 11.0;
  init_msg.twist.angular.y = 13.0;
  init_msg.twist.angular.z = 17.0;

  init_msg.accel.linear.x = 19.0;
  init_msg.accel.linear.y = 23.0;
  init_msg.accel.linear.z = 29.0;
  init_msg.accel.angular.x = 31.0;
  init_msg.accel.angular.y = 37.0;
  init_msg.accel.angular.z = 41.0;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_ACTION_STATUS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_EntityStatus)
{
  traffic_simulator_msgs::msg::EntityStatus src_msg;
  traffic_simulator_msgs::EntityStatus dst_proto;

  src_msg.name = "test";
  src_msg.time = 3.0;

  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 5.0;
  box_msg.center.y = 7.0;
  box_msg.center.z = 11.0;
  box_msg.dimensions.x = 13.0;
  box_msg.dimensions.y = 17.0;
  box_msg.dimensions.z = 19.0;
  src_msg.bounding_box = box_msg;

  traffic_simulator_msgs::msg::ActionStatus action_msg;
  action_msg.current_action = "test";
  action_msg.twist.linear.x = 23.0;
  action_msg.twist.linear.y = 29.0;
  action_msg.twist.linear.z = 31.0;
  action_msg.twist.angular.x = 37.0;
  action_msg.twist.angular.y = 41.0;
  action_msg.twist.angular.z = 43.0;
  action_msg.accel.linear.x = 47.0;
  action_msg.accel.linear.y = 53.0;
  action_msg.accel.linear.z = 59.0;
  action_msg.accel.angular.x = 61.0;
  action_msg.accel.angular.y = 67.0;
  action_msg.accel.angular.z = 71.0;
  src_msg.action_status = action_msg;

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 73.0;
  pose_msg.position.y = 79.0;
  pose_msg.position.z = 83.0;
  pose_msg.orientation.x = 89.0;
  pose_msg.orientation.y = 97.0;
  pose_msg.orientation.z = 101.0;
  pose_msg.orientation.w = 103.0;
  src_msg.pose = pose_msg;

  traffic_simulator_msgs::msg::LaneletPose lanelet_pose_msg;
  lanelet_pose_msg.lanelet_id = 107;
  lanelet_pose_msg.s = 109.0;
  lanelet_pose_msg.offset = 113.0;
  lanelet_pose_msg.rpy.x = 127.0;
  lanelet_pose_msg.rpy.y = 131.0;
  lanelet_pose_msg.rpy.z = 137.0;
  src_msg.lanelet_pose = lanelet_pose_msg;
  src_msg.lanelet_pose_valid = true;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_ENTITY_STATUS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_EntityStatus)
{
  traffic_simulator_msgs::msg::EntityStatus init_msg;
  traffic_simulator_msgs::EntityStatus src_proto;
  traffic_simulator_msgs::msg::EntityStatus dst_msg;

  init_msg.name = "test";
  init_msg.time = 3.0;

  traffic_simulator_msgs::msg::BoundingBox box_msg;
  box_msg.center.x = 5.0;
  box_msg.center.y = 7.0;
  box_msg.center.z = 11.0;
  box_msg.dimensions.x = 13.0;
  box_msg.dimensions.y = 17.0;
  box_msg.dimensions.z = 19.0;
  init_msg.bounding_box = box_msg;

  traffic_simulator_msgs::msg::ActionStatus action_msg;
  action_msg.current_action = "test";
  action_msg.twist.linear.x = 23.0;
  action_msg.twist.linear.y = 29.0;
  action_msg.twist.linear.z = 31.0;
  action_msg.twist.angular.x = 37.0;
  action_msg.twist.angular.y = 41.0;
  action_msg.twist.angular.z = 43.0;
  action_msg.accel.linear.x = 47.0;
  action_msg.accel.linear.y = 53.0;
  action_msg.accel.linear.z = 59.0;
  action_msg.accel.angular.x = 61.0;
  action_msg.accel.angular.y = 67.0;
  action_msg.accel.angular.z = 71.0;
  init_msg.action_status = action_msg;

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 73.0;
  pose_msg.position.y = 79.0;
  pose_msg.position.z = 83.0;
  pose_msg.orientation.x = 89.0;
  pose_msg.orientation.y = 97.0;
  pose_msg.orientation.z = 101.0;
  pose_msg.orientation.w = 103.0;
  init_msg.pose = pose_msg;

  traffic_simulator_msgs::msg::LaneletPose lanelet_pose_msg;
  lanelet_pose_msg.lanelet_id = 107;
  lanelet_pose_msg.s = 109.0;
  lanelet_pose_msg.offset = 113.0;
  lanelet_pose_msg.rpy.x = 127.0;
  lanelet_pose_msg.rpy.y = 131.0;
  lanelet_pose_msg.rpy.z = 137.0;
  init_msg.lanelet_pose = lanelet_pose_msg;
  init_msg.lanelet_pose_valid = true;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_ENTITY_STATUS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_SentEntityStatus)
{
  traffic_simulator_msgs::msg::EntityStatus src_msg;
  simulation_api_schema::EntityStatus dst_proto;

  src_msg.name = "test";
  src_msg.time = 3.0;

  traffic_simulator_msgs::msg::ActionStatus action_msg;
  action_msg.current_action = "test";
  action_msg.twist.linear.x = 5.0;
  action_msg.twist.linear.y = 7.0;
  action_msg.twist.linear.z = 11.0;
  action_msg.twist.angular.x = 13.0;
  action_msg.twist.angular.y = 17.0;
  action_msg.twist.angular.z = 19.0;
  action_msg.accel.linear.x = 23.0;
  action_msg.accel.linear.y = 29.0;
  action_msg.accel.linear.z = 31.0;
  action_msg.accel.angular.x = 37.0;
  action_msg.accel.angular.y = 41.0;
  action_msg.accel.angular.z = 43.0;
  src_msg.action_status = action_msg;

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 47.0;
  pose_msg.position.y = 53.0;
  pose_msg.position.z = 59.0;
  pose_msg.orientation.x = 61.0;
  pose_msg.orientation.y = 67.0;
  pose_msg.orientation.z = 71.0;
  pose_msg.orientation.w = 73.0;
  src_msg.pose = pose_msg;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_SENT_ENTITY_STATUS_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_SentEntityStatus)
{
  traffic_simulator_msgs::msg::EntityStatus init_msg;
  simulation_api_schema::EntityStatus src_proto;
  traffic_simulator_msgs::msg::EntityStatus dst_msg;

  init_msg.name = "test";
  init_msg.time = 3.0;

  traffic_simulator_msgs::msg::ActionStatus action_msg;
  action_msg.current_action = "test";
  action_msg.twist.linear.x = 5.0;
  action_msg.twist.linear.y = 7.0;
  action_msg.twist.linear.z = 11.0;
  action_msg.twist.angular.x = 13.0;
  action_msg.twist.angular.y = 17.0;
  action_msg.twist.angular.z = 19.0;
  action_msg.accel.linear.x = 23.0;
  action_msg.accel.linear.y = 29.0;
  action_msg.accel.linear.z = 31.0;
  action_msg.accel.angular.x = 37.0;
  action_msg.accel.angular.y = 41.0;
  action_msg.accel.angular.z = 43.0;
  init_msg.action_status = action_msg;

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 47.0;
  pose_msg.position.y = 53.0;
  pose_msg.position.z = 59.0;
  pose_msg.orientation.x = 61.0;
  pose_msg.orientation.y = 67.0;
  pose_msg.orientation.z = 71.0;
  pose_msg.orientation.w = 73.0;
  init_msg.pose = pose_msg;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_SENT_ENTITY_STATUS_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Time)
{
  builtin_interfaces::msg::Time src_msg;
  builtin_interfaces::Time dst_proto;

  src_msg.nanosec = 3;
  src_msg.sec = 5;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_TIME_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Time)
{
  builtin_interfaces::msg::Time init_msg;
  builtin_interfaces::Time src_proto;
  builtin_interfaces::msg::Time dst_msg;

  init_msg.nanosec = 3;
  init_msg.sec = 5;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_TIME_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Duration)
{
  builtin_interfaces::msg::Duration src_msg;
  builtin_interfaces::Duration dst_proto;

  src_msg.nanosec = 3;
  src_msg.sec = 5;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_DURATION_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Duration)
{
  builtin_interfaces::msg::Duration init_msg;
  builtin_interfaces::Duration src_proto;
  builtin_interfaces::msg::Duration dst_msg;

  init_msg.nanosec = 3;
  init_msg.sec = 5;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_DURATION_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Header)
{
  std_msgs::msg::Header src_msg;
  std_msgs::Header dst_proto;

  src_msg.frame_id = "test";
  src_msg.stamp.nanosec = 3;
  src_msg.stamp.sec = 5;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_HEADER_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Header)
{
  std_msgs::msg::Header init_msg;
  std_msgs::Header src_proto;
  std_msgs::msg::Header dst_msg;

  init_msg.frame_id = "test";
  init_msg.stamp.nanosec = 3;
  init_msg.stamp.sec = 5;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_HEADER_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_Clock)
{
  rosgraph_msgs::msg::Clock src_msg;
  rosgraph_msgs::Clock dst_proto;

  src_msg.clock.nanosec = 3;
  src_msg.clock.sec = 5;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_CLOCK_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_Clock)
{
  rosgraph_msgs::msg::Clock init_msg;
  rosgraph_msgs::Clock src_proto;
  rosgraph_msgs::msg::Clock dst_msg;

  init_msg.clock.nanosec = 3;
  init_msg.clock.sec = 5;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_CLOCK_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_AckermannControlCommand)
{
  autoware_auto_control_msgs::msg::AckermannControlCommand src_msg;
  autoware_auto_control_msgs::AckermannControlCommand dst_proto;

  src_msg.longitudinal.acceleration = 3.0;
  src_msg.lateral.steering_tire_angle = 5.0;
  src_msg.lateral.steering_tire_rotation_rate = 7.0;
  src_msg.longitudinal.speed = 11.0;

  simulation_interface::toProto(src_msg, dst_proto);

  EXPECT_CONTROL_COMMAND_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_AckermannControlCommand)
{
  autoware_auto_control_msgs::msg::AckermannControlCommand init_msg;
  autoware_auto_control_msgs::AckermannControlCommand src_proto;
  autoware_auto_control_msgs::msg::AckermannControlCommand dst_msg;

  init_msg.longitudinal.acceleration = 3.0;
  init_msg.lateral.steering_tire_angle = 5.0;
  init_msg.lateral.steering_tire_rotation_rate = 7.0;
  init_msg.longitudinal.speed = 11.0;

  simulation_interface::toProto(init_msg, src_proto);
  simulation_interface::toMsg(src_proto, dst_msg);

  EXPECT_CONTROL_COMMAND_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_EntityTypeEgo)
{
  traffic_simulator_msgs::msg::EntityType src_msg;
  traffic_simulator_msgs::EntityType dst_proto;
  src_msg.type = traffic_simulator_msgs::msg::EntityType::EGO;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO);
}

TEST(Conversion, toMsg_EntityTypeEgo)
{
  traffic_simulator_msgs::msg::EntityType init_msg;
  traffic_simulator_msgs::EntityType src_proto;
  traffic_simulator_msgs::msg::EntityType dst_msg;
  init_msg.type = traffic_simulator_msgs::msg::EntityType::EGO;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.type, traffic_simulator_msgs::msg::EntityType::EGO);
}

TEST(Conversion, toProto_EntityTypeVehicle)
{
  traffic_simulator_msgs::msg::EntityType src_msg;
  traffic_simulator_msgs::EntityType dst_proto;
  src_msg.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_VEHICLE);
}

TEST(Conversion, toMsg_EntityTypeVehicle)
{
  traffic_simulator_msgs::msg::EntityType init_msg;
  traffic_simulator_msgs::EntityType src_proto;
  traffic_simulator_msgs::msg::EntityType dst_msg;
  init_msg.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.type, traffic_simulator_msgs::msg::EntityType::VEHICLE);
}

TEST(Conversion, toProto_EntityTypePedestrian)
{
  traffic_simulator_msgs::msg::EntityType src_msg;
  traffic_simulator_msgs::EntityType dst_proto;
  src_msg.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_PEDESTRIAN);
}

TEST(Conversion, toMsg_EntityTypePedestrian)
{
  traffic_simulator_msgs::msg::EntityType init_msg;
  traffic_simulator_msgs::EntityType src_proto;
  traffic_simulator_msgs::msg::EntityType dst_msg;
  init_msg.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.type, traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
}

TEST(Conversion, toProto_EntityTypeMiscObject)
{
  traffic_simulator_msgs::msg::EntityType src_msg;
  traffic_simulator_msgs::EntityType dst_proto;
  src_msg.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.type(), traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_MISC_OBJECT);
}

TEST(Conversion, toMsg_EntityTypeMiscObject)
{
  traffic_simulator_msgs::msg::EntityType init_msg;
  traffic_simulator_msgs::EntityType src_proto;
  traffic_simulator_msgs::msg::EntityType dst_msg;
  init_msg.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.type, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT);
}

TEST(Conversion, toProto_EntitySubtypeUnknown)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN);
}

TEST(Conversion, toMsg_EntitySubtypeUnknown)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN);
}

TEST(Conversion, toProto_EntitySubtypeCar)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR);
}

TEST(Conversion, toMsg_EntitySubtypeCar)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::CAR);
}

TEST(Conversion, toProto_EntitySubtypeTruck)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::TRUCK;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRUCK);
}

TEST(Conversion, toMsg_EntitySubtypeTruck)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::TRUCK;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::TRUCK);
}

TEST(Conversion, toProto_EntitySubtypeBus)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::BUS;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BUS);
}

TEST(Conversion, toMsg_EntitySubtypeBus)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::BUS;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::BUS);
}

TEST(Conversion, toProto_EntitySubtypeTrailer)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::TRAILER;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRAILER);
}

TEST(Conversion, toMsg_EntitySubtypeTrailer)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::TRAILER;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::TRAILER);
}

TEST(Conversion, toProto_EntitySubtypeMotorcycle)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_MOTORCYCLE);
}

TEST(Conversion, toMsg_EntitySubtypeMotorcycle)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE);
}

TEST(Conversion, toProto_EntitySubtypeBicycle)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::BICYCLE;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BICYCLE);
}

TEST(Conversion, toMsg_EntitySubtypeBicycle)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::BICYCLE;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::BICYCLE);
}

TEST(Conversion, toProto_EntitySubtypePedestrian)
{
  traffic_simulator_msgs::msg::EntitySubtype src_msg;
  traffic_simulator_msgs::EntitySubtype dst_proto;
  src_msg.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(
    dst_proto.value(), traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_PEDESTRIAN);
}

TEST(Conversion, toMsg_EntitySubtypePedestrian)
{
  traffic_simulator_msgs::msg::EntitySubtype init_msg;
  traffic_simulator_msgs::EntitySubtype src_proto;
  traffic_simulator_msgs::msg::EntitySubtype dst_msg;
  init_msg.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.value, traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN);
}

TEST(Conversion, toProto_LaneletPose)
{
  traffic_simulator_msgs::msg::LaneletPose src_msg;
  traffic_simulator_msgs::LaneletPose dst_proto;
  src_msg.lanelet_id = 3;
  src_msg.s = 5.0;
  src_msg.offset = 7.0;
  src_msg.rpy.x = 11.0;
  src_msg.rpy.y = 13.0;
  src_msg.rpy.z = 17.0;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_LANELET_POSE_EQ(src_msg, dst_proto);
}

TEST(Conversion, toMsg_LaneletPose)
{
  traffic_simulator_msgs::msg::LaneletPose init_msg;
  traffic_simulator_msgs::LaneletPose src_proto;
  traffic_simulator_msgs::msg::LaneletPose dst_msg;
  init_msg.lanelet_id = 3;
  init_msg.s = 5.0;
  init_msg.offset = 7.0;
  init_msg.rpy.x = 11.0;
  init_msg.rpy.y = 13.0;
  init_msg.rpy.z = 17.0;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_LANELET_POSE_EQ(dst_msg, src_proto);
}

TEST(Conversion, toProto_GearCommandNone)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::NONE);
}

TEST(Conversion, toMsg_GearCommandNone)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::NONE);
}

TEST(Conversion, toProto_GearCommandNeutral)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::NEUTRAL);
}

TEST(Conversion, toMsg_GearCommandNeutral)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL);
}

TEST(Conversion, toProto_GearCommandDrive)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE);
}

TEST(Conversion, toMsg_GearCommandDrive)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE);
}

TEST(Conversion, toProto_GearCommandDrive2)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_2;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_2);
}

TEST(Conversion, toMsg_GearCommandDrive2)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_2;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_2);
}

TEST(Conversion, toProto_GearCommandDrive3)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_3;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_3);
}

TEST(Conversion, toMsg_GearCommandDrive3)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_3;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_3);
}

TEST(Conversion, toProto_GearCommandDrive4)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_4;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_4);
}

TEST(Conversion, toMsg_GearCommandDrive4)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_4;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_4);
}

TEST(Conversion, toProto_GearCommandDrive5)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_5;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_5);
}

TEST(Conversion, toMsg_GearCommandDrive5)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_5;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_5);
}

TEST(Conversion, toProto_GearCommandDrive6)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_6;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_6);
}

TEST(Conversion, toMsg_GearCommandDrive6)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_6;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_6);
}

TEST(Conversion, toProto_GearCommandDrive7)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_7;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_7);
}

TEST(Conversion, toMsg_GearCommandDrive7)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_7;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_7);
}

TEST(Conversion, toProto_GearCommandDrive8)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_8;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_8);
}

TEST(Conversion, toMsg_GearCommandDrive8)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_8;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_8);
}

TEST(Conversion, toProto_GearCommandDrive9)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_9;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_9);
}

TEST(Conversion, toMsg_GearCommandDrive9)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_9;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_9);
}

TEST(Conversion, toProto_GearCommandDrive10)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_10;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_10);
}

TEST(Conversion, toMsg_GearCommandDrive10)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_10;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_10);
}

TEST(Conversion, toProto_GearCommandDrive11)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_11;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_11);
}

TEST(Conversion, toMsg_GearCommandDrive11)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_11;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_11);
}

TEST(Conversion, toProto_GearCommandDrive12)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_12;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_12);
}

TEST(Conversion, toMsg_GearCommandDrive12)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_12;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_12);
}

TEST(Conversion, toProto_GearCommandDrive13)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_13;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_13);
}

TEST(Conversion, toMsg_GearCommandDrive13)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_13;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_13);
}

TEST(Conversion, toProto_GearCommandDrive14)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_14;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_14);
}

TEST(Conversion, toMsg_GearCommandDrive14)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_14;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_14);
}

TEST(Conversion, toProto_GearCommandDrive15)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_15;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_15);
}

TEST(Conversion, toMsg_GearCommandDrive15)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_15;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_15);
}

TEST(Conversion, toProto_GearCommandDrive16)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_16;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_16);
}

TEST(Conversion, toMsg_GearCommandDrive16)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_16;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_16);
}

TEST(Conversion, toProto_GearCommandDrive17)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_17;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_17);
}

TEST(Conversion, toMsg_GearCommandDrive17)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_17;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_17);
}

TEST(Conversion, toProto_GearCommandDrive18)
{
  autoware_auto_vehicle_msgs::msg::GearCommand src_msg;
  autoware_auto_vehicle_msgs::GearCommand dst_proto;
  src_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_18;
  EXPECT_NO_THROW(simulation_interface::toProto(src_msg, dst_proto));
  EXPECT_EQ(dst_proto.command(), autoware_auto_vehicle_msgs::GearCommand_Constants::DRIVE_18);
}

TEST(Conversion, toMsg_GearCommandDrive18)
{
  autoware_auto_vehicle_msgs::msg::GearCommand init_msg;
  autoware_auto_vehicle_msgs::GearCommand src_proto;
  autoware_auto_vehicle_msgs::msg::GearCommand dst_msg;
  init_msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_18;
  EXPECT_NO_THROW(simulation_interface::toProto(init_msg, src_proto));
  EXPECT_NO_THROW(simulation_interface::toMsg(src_proto, dst_msg));
  EXPECT_EQ(dst_msg.command, autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_18);
}