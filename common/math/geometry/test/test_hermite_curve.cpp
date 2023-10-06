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

#include <geometry/spline/hermite_curve.hpp>

constexpr double EPS = 1e-3;

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Vector3 makeVector(double x, double y, double z = 0)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

geometry_msgs::msg::Pose makePose(
  double x, double y, double z = 0,
  geometry_msgs::msg::Quaternion q = geometry_msgs::msg::Quaternion())
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = q;
  return p;
}

TEST(HermiteCurveTest, CheckCollisionToLine)
{
  geometry_msgs::msg::Pose start_pose = makePose(0, 0);
  geometry_msgs::msg::Pose goal_pose = makePose(1, 0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1, 0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1, 0);
  math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
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
  geometry_msgs::msg::Pose p;
  p.position = makePoint(0.1, 0);
  EXPECT_TRUE(curve.getSValue(p, 1, true));
  EXPECT_NEAR(curve.getSValue(p, 1, true).value(), 0.1, EPS);
  {
    geometry_msgs::msg::Point start = makePoint(0.1, 1.0);
    geometry_msgs::msg::Point goal = makePoint(0.1, -1.0);
    auto collision_s = curve.getCollisionPointIn2D(start, goal);
    EXPECT_FALSE(curve.getCollisionPointIn2D({}));
    EXPECT_FALSE(curve.getCollisionPointIn2D({start}));
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.1);
    }
  }
  {
    geometry_msgs::msg::Point start = makePoint(0.1, 1.0);
    geometry_msgs::msg::Point goal = makePoint(0.2, -1.0);
    auto collision_s = curve.getCollisionPointIn2D(start, goal);
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.15);
    }
  }
}

TEST(HermiteCurveTest, getNewtonMethodStepSize) {}

TEST(HermiteCurveTest, CheckNormalVector)
{
  {  //p(0,0) v(1,0)-> p(1,1) v(0,1)
    geometry_msgs::msg::Pose start_pose = makePose(0, 0);
    geometry_msgs::msg::Pose goal_pose = makePose(1, 1);
    geometry_msgs::msg::Vector3 start_vec = makeVector(1, 0);
    geometry_msgs::msg::Vector3 goal_vec = makeVector(0, 1);
    math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm =
      std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1 / std::sqrt(2));
  }
  {  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
    geometry_msgs::msg::Pose start_pose = makePose(0, 0);
    geometry_msgs::msg::Pose goal_pose = makePose(1, -1);
    geometry_msgs::msg::Vector3 start_vec = makeVector(1, 0);
    geometry_msgs::msg::Vector3 goal_vec = makeVector(0, -1);
    math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm =
      std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1 / std::sqrt(2));
  }
  {  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
    geometry_msgs::msg::Pose start_pose = makePose(1, 1);
    geometry_msgs::msg::Pose goal_pose = makePose(0, 0);
    geometry_msgs::msg::Vector3 start_vec = makeVector(0, -1);
    geometry_msgs::msg::Vector3 goal_vec = makeVector(-1, 0);
    math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm =
      std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1 / std::sqrt(2));
    EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, -1 / std::sqrt(2));
  }
  {  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
    geometry_msgs::msg::Pose start_pose = makePose(1, -1);
    geometry_msgs::msg::Pose goal_pose = makePose(0, 0);
    geometry_msgs::msg::Vector3 start_vec = makeVector(0, 1);
    geometry_msgs::msg::Vector3 goal_vec = makeVector(-1, 0);
    math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
    double norm =
      std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
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
