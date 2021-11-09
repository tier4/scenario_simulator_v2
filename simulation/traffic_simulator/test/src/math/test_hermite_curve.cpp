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

#include <traffic_simulator/math/hermite_curve.hpp>

TEST(HermiteCurveTest, CheckCollisionToLine)
{
  geometry_msgs::msg::Pose start_pose, goal_pose;
  geometry_msgs::msg::Vector3 start_vec, goal_vec;
  goal_pose.position.x = 1;
  start_vec.x = 1;
  goal_vec.x = 1;
  traffic_simulator::math::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
  EXPECT_DOUBLE_EQ(curve.getLength(), 1);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.1, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.2, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.3, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.4, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.5, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.6, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.7, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.8, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.9, true), 0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(1.0, true), 0);
  EXPECT_DOUBLE_EQ(curve.getPoint(0.5, false).x, 0.5);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x, 1);
  EXPECT_DOUBLE_EQ(curve.getMaximum2DCurvature(), 0);
  geometry_msgs::msg::Point p;
  p.x = 0.1;
  p.y = 0;
  p.z = 0;
  EXPECT_TRUE(curve.getSValue(p, true, 30, 0.001));
  EXPECT_TRUE(
    (curve.getSValue(p, true, 30, 0.001).get() > 0.099) &&
    (curve.getSValue(p, true, 30, 0.001).get() < 0.101));
  {
    geometry_msgs::msg::Point start;
    start.x = 0.1;
    start.y = 1.0;
    geometry_msgs::msg::Point goal;
    goal.x = 0.1;
    goal.y = -1.0;
    auto collision_s = curve.getCollisionPointIn2D(start, goal);
    EXPECT_FALSE(curve.getCollisionPointIn2D({}));
    EXPECT_FALSE(curve.getCollisionPointIn2D({start}));
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.get(), 0.1);
    }
  }
  {
    geometry_msgs::msg::Point start;
    start.x = 0.1;
    start.y = 1.0;
    geometry_msgs::msg::Point goal;
    goal.x = 0.2;
    goal.y = -1.0;
    auto collision_s = curve.getCollisionPointIn2D(start, goal);
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.get(), 0.15);
    }
  }
}

TEST(HermiteCurveTest, getNewtonMethodStepSize) {}

TEST(HermiteCurveTest, CheckNormalVector)
{
  {  //p(0,0) v(1,0)-> p(1,1) v(0,1)
    geometry_msgs::msg::Pose start_pose, goal_pose;
    geometry_msgs::msg::Vector3 start_vec, goal_vec;
    start_pose.position.x = 0;
    start_pose.position.y = 0;
    goal_pose.position.x = 1;
    goal_pose.position.y = 1;
    start_vec.x = 1;
    start_vec.y = 0;
    goal_vec.x = 0;
    goal_vec.y = 1;
    traffic_simulator::math::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm = std::sqrt(
      curve.getTangentVector(0.5, false).x * curve.getTangentVector(0.5, false).x +
      curve.getTangentVector(0.5, false).y * curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1 / std::sqrt(2));
  }
  {  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
    geometry_msgs::msg::Pose start_pose, goal_pose;
    geometry_msgs::msg::Vector3 start_vec, goal_vec;
    start_pose.position.x = 0;
    start_pose.position.y = 0;
    goal_pose.position.x = 1;
    goal_pose.position.y = -1;
    start_vec.x = 1;
    start_vec.y = 0;
    goal_vec.x = 0;
    goal_vec.y = -1;
    traffic_simulator::math::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm = std::sqrt(
      curve.getTangentVector(0.5, false).x * curve.getTangentVector(0.5, false).x +
      curve.getTangentVector(0.5, false).y * curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1 / std::sqrt(2));
  }
  {  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
    geometry_msgs::msg::Pose start_pose, goal_pose;
    geometry_msgs::msg::Vector3 start_vec, goal_vec;
    start_pose.position.x = 1;
    start_pose.position.y = 1;
    goal_pose.position.x = 0;
    goal_pose.position.y = 0;
    start_vec.x = 0;
    start_vec.y = -1;
    goal_vec.x = -1;
    goal_vec.y = 0;
    traffic_simulator::math::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm = std::sqrt(
      curve.getTangentVector(0.5, false).x * curve.getTangentVector(0.5, false).x +
      curve.getTangentVector(0.5, false).y * curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, -1 / std::sqrt(2));
  }
  {  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
    geometry_msgs::msg::Pose start_pose, goal_pose;
    geometry_msgs::msg::Vector3 start_vec, goal_vec;
    start_pose.position.x = 1;
    start_pose.position.y = -1;
    goal_pose.position.x = 0;
    goal_pose.position.y = 0;
    start_vec.x = 0;
    start_vec.y = 1;
    goal_vec.x = -1;
    goal_vec.y = 0;
    traffic_simulator::math::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm = std::sqrt(
      curve.getTangentVector(0.5, false).x * curve.getTangentVector(0.5, false).x +
      curve.getTangentVector(0.5, false).y * curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, -1 / std::sqrt(2));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
