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

#include <gtest/gtest.h>

#include <geometry/vector3/vector3.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../expect_eq_macros.hpp"

/**
 * @note Test basic functionality. Test construction correctness of an action status.
 */
TEST(helper, constructActionStatus)
{
  traffic_simulator_msgs::msg::ActionStatus actual_action_status;
  actual_action_status.twist.linear.x = 2.0;
  actual_action_status.twist.angular.z = 3.0;
  actual_action_status.accel.linear.x = 5.0;
  actual_action_status.accel.angular.z = 7.0;

  const auto result_action_status =
    traffic_simulator::helper::constructActionStatus(2.0, 3.0, 5.0, 7.0);

  EXPECT_ACTION_STATUS_EQ(result_action_status, actual_action_status);
}

/**
 * @note Test basic functionality. Test construction correctness of a lanelet pose.
 */
TEST(helper, constructLaneletPose)
{
  const auto actual_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator::LaneletPose>()
      .lanelet_id(11LL)
      .s(13.0)
      .offset(17.0)
      .rpy(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(19.0).y(23.0).z(29.0));

  const auto result_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(11LL, 13.0, 17.0, 19.0, 23.0, 29.0);

  std::stringstream ss;
  ss << result_lanelet_pose;
  EXPECT_LANELET_POSE_EQ(result_lanelet_pose, actual_lanelet_pose);
  EXPECT_STREQ(ss.str().c_str(), "lanelet id : 11\ns : 13");
}

/**
 * @note Test basic functionality. Test construction correctness of RPY vector.
 */
TEST(helper, constructRPY)
{
  const auto vec = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(31.0).y(37.0).z(41.0);
  const auto rpy = traffic_simulator::helper::constructRPY(31.0, 37.0, 41.0);
  EXPECT_VECTOR3_EQ(rpy, vec);
}

/**
 * @note Test basic functionality. Test construction correctness of a quaternion.
 */
TEST(helper, constructRPYfromQuaternion)
{
  {
    const auto default_vec = geometry_msgs::msg::Vector3();
    const auto default_rpy =
      traffic_simulator::helper::constructRPYfromQuaternion(geometry_msgs::msg::Quaternion());
    EXPECT_VECTOR3_EQ(default_rpy, default_vec);
  }
  {
    const auto vec = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                       .x(-M_PI / 3.0)
                       .y(-M_PI / 6.0)
                       .z(M_PI / 2.0);
    const auto rpy = traffic_simulator::helper::constructRPYfromQuaternion(
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(-0.183).y(-0.5).z(0.5).w(0.683));
    EXPECT_VECTOR3_NEAR(rpy, vec, 1.0e-3);
  }
}

/**
 * @note Test basic functionality. Test construction correctness of a pose.
 */
TEST(helper, constructPose)
{
  const auto actual_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(43.0).y(47.0).z(53.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(-0.183).y(-0.5).z(0.5).w(0.683));

  const auto result_pose = traffic_simulator::helper::constructPose(
    43.0, 47.0, 53.0, -M_PI / 3.0, -M_PI / 6.0, M_PI / 2.0);

  EXPECT_POSE_NEAR(result_pose, actual_pose, 1.0e-3);
}

/**
 * @note Test basic functionality. Test construction correctness of
 * a lidar sensor with both VLP16 and VLP32 configurations.
 */
TEST(helper, constructLidarConfiguration)
{
  EXPECT_NO_THROW(traffic_simulator::helper::constructLidarConfiguration(
    traffic_simulator::helper::LidarType::VLP16, "ego", "test"));
  EXPECT_NO_THROW(traffic_simulator::helper::constructLidarConfiguration(
    traffic_simulator::helper::LidarType::VLP32, "ego", "test"));
}

/**
 * @note Test basic functionality. Test construction correctness of 
 * a detection sensor with a given configuration.
 */
TEST(helper, constructDetectionSensorConfiguration)
{
  simulation_api_schema::DetectionSensorConfiguration actual_configuration;
  actual_configuration.set_architecture_type("test");
  actual_configuration.set_entity("ego");
  actual_configuration.set_update_duration(3.0);

  const auto result_configuration =
    traffic_simulator::helper::constructDetectionSensorConfiguration("ego", "test", 3.0);

  EXPECT_DETECTION_SENSOR_CONFIGURATION_EQ(result_configuration, actual_configuration);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
