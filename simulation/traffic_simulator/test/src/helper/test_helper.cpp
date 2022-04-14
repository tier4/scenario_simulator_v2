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

#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../expect_eq_macros.hpp"

TEST(HELPER, RPY)
{
  const auto rpy = traffic_simulator::helper::constructRPY(1, 2, 1);
  geometry_msgs::msg::Vector3 vec;
  vec.x = 1.0;
  vec.y = 2.0;
  vec.z = 1.0;
  EXPECT_VECTOR3_EQ(rpy, vec);
}

TEST(HELPER, QUATERNION)
{
  const auto rpy =
    traffic_simulator::helper::constructRPYfromQuaternion(geometry_msgs::msg::Quaternion());
  EXPECT_VECTOR3_EQ(rpy, geometry_msgs::msg::Vector3());
}

TEST(HELPER, POSE)
{
  const auto pose = traffic_simulator::helper::constructPose(1, 1, 1, 0, 0, 0);
  geometry_msgs::msg::Pose expect_pose;
  expect_pose.position.x = 1;
  expect_pose.position.y = 1;
  expect_pose.position.z = 1;
  EXPECT_POSE_EQ(pose, expect_pose);
}

TEST(HELPER, LANELET_POSE)
{
  const auto lanelet_pose = traffic_simulator::helper::constructLaneletPose(5, 10, 2, 0, 0, 0);
  traffic_simulator_msgs::msg::LaneletPose expected_pose;
  expected_pose.lanelet_id = 5;
  expected_pose.s = 10.0;
  expected_pose.offset = 2.0;
  expected_pose.rpy.x = 0;
  expected_pose.rpy.y = 0;
  expected_pose.rpy.z = 0;
  EXPECT_LANELET_POSE_EQ(lanelet_pose, expected_pose);
  std::stringstream ss;
  ss << lanelet_pose;
  EXPECT_STREQ(ss.str().c_str(), "lanelet id : 5\ns : 10");
}

TEST(HELPER, ACTION_STATUS)
{
  const auto action_status = traffic_simulator::helper::constructActionStatus(3, 4, 5, 1);
  traffic_simulator_msgs::msg::ActionStatus expect_action_status;
  expect_action_status.twist.linear.x = 3;
  expect_action_status.twist.angular.z = 4;
  expect_action_status.accel.linear.x = 5;
  expect_action_status.accel.angular.z = 1;
  EXPECT_ACTION_STATUS_EQ(action_status, expect_action_status);
}

TEST(HELPER, DETECTION_SENSOR_CONFIGURATION)
{
  const auto configuration =
    traffic_simulator::helper::constructDetectionSensorConfiguration("ego", "test", 3);
  simulation_api_schema::DetectionSensorConfiguration expect_configuration;
  expect_configuration.set_architecture_type("test");
  expect_configuration.set_entity("ego");
  expect_configuration.set_update_duration(3.0);
  EXPECT_DETECTION_SENSOR_CONFIGURATION_EQ(configuration, expect_configuration);
}

TEST(HELPER, LIDAR_SENSOR_CONFIGURATION)
{
  EXPECT_NO_THROW(traffic_simulator::helper::constructLidarConfiguration(
    traffic_simulator::helper::LidarType::VLP16, "ego", "test"));
  EXPECT_NO_THROW(traffic_simulator::helper::constructLidarConfiguration(
    traffic_simulator::helper::LidarType::VLP32, "ego", "test"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
