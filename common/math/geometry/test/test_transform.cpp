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
#include <quaternion_operation/quaternion_operation.h>

#include <geometry/transform.hpp>

#include "expect_eq_macros.hpp"

constexpr double EPS = 1e-3;

geometry_msgs::msg::Pose filled_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion([]() {
    geometry_msgs::msg::Vector3 v;
    v.x = 90 * M_PI / 180.0;
    v.y = 0.0;
    v.z = 0.0;
    return v;
  }());
  return pose;
}

TEST(Transform, getRelativePoseDifferent)
{
  auto pose0 = filled_pose();
  auto pose1 = geometry_msgs::msg::Pose();
  pose1.position.x = 3;

  auto ans = filled_pose();
  ans.position.x = 2.0;
  ans.position.y = -3.0;
  ans.position.z = 2.0;
  ans.orientation.x = -0.707;

  EXPECT_POSE_NEAR(math::geometry::getRelativePose(pose0, pose1), ans, EPS);
}

TEST(Transform, getRelativePoseIdentical)
{
  auto pose = filled_pose();
  EXPECT_POSE_EQ(math::geometry::getRelativePose(pose, pose), geometry_msgs::msg::Pose());
}

TEST(Transform, transformPointRealPose)
{
  auto pose = filled_pose();
  auto point = geometry_msgs::msg::Point();
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  auto ans = geometry_msgs::msg::Point();
  ans.x = 2.0;
  ans.y = -1.0;
  ans.z = 5.0;
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, point), ans);
}

TEST(Transform, transformPointNoPose)
{
  auto pose = geometry_msgs::msg::Pose();
  auto point = geometry_msgs::msg::Point();
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, point), point);
}

TEST(Transform, transformPointRealSensorPose)
{
  auto pose = filled_pose();
  auto sensor_pose = filled_pose();
  sensor_pose.orientation = geometry_msgs::msg::Quaternion();
  auto point = geometry_msgs::msg::Point();
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  auto ans = geometry_msgs::msg::Point();
  ans.x = 1.0;
  ans.y = -3.0;
  ans.z = 2.0;
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, sensor_pose, point), ans);
}

TEST(Transform, transformPointNoSensorPose)
{
  auto pose = filled_pose();
  auto sensor_pose = geometry_msgs::msg::Pose();
  auto point = geometry_msgs::msg::Point();
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  auto ans = geometry_msgs::msg::Point();
  ans.x = 2.0;
  ans.y = -1.0;
  ans.z = 5.0;
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, sensor_pose, point), ans);
}

TEST(Transform, transformPointsRealPose)
{
  auto pose = filled_pose();
  std::vector<geometry_msgs::msg::Point> points(2);
  points[0].x = 1.0;
  points[0].y = 2.0;
  points[0].z = 3.0;
  points[1].x = 1.0;
  points[1].y = 4.0;
  points[1].z = 3.0;

  auto ans0 = geometry_msgs::msg::Point();
  ans0.x = 2.0;
  ans0.y = -1.0;
  ans0.z = 5.0;
  auto ans1 = geometry_msgs::msg::Point();
  ans1.x = 2.0;
  ans1.y = -1.0;
  ans1.z = 7.0;

  std::vector<geometry_msgs::msg::Point> ans = math::geometry::transformPoints(pose, points);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsNoPose)
{
  auto pose = geometry_msgs::msg::Pose();
  std::vector<geometry_msgs::msg::Point> points(2);
  points[0].x = 1.0;
  points[0].y = 2.0;
  points[0].z = 3.0;
  points[1].x = 1.0;
  points[1].y = 4.0;
  points[1].z = 3.0;

  std::vector<geometry_msgs::msg::Point> ans = math::geometry::transformPoints(pose, points);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], points[0], EPS);
  EXPECT_POINT_NEAR(ans[1], points[1], EPS);
}

TEST(Transform, transformPointsEmptyVector)
{
  auto pose = filled_pose();
  std::vector<geometry_msgs::msg::Point> points;

  std::vector<geometry_msgs::msg::Point> ans;
  EXPECT_NO_THROW(ans = math::geometry::transformPoints(pose, points));
  EXPECT_EQ(ans.size(), size_t(0));
}

TEST(Transform, transformPointsRealSensorPose)
{
  auto pose = filled_pose();
  auto sensor_pose = filled_pose();
  sensor_pose.orientation = geometry_msgs::msg::Quaternion();
  std::vector<geometry_msgs::msg::Point> points(2);
  points[0].x = 1.0;
  points[0].y = 2.0;
  points[0].z = 3.0;
  points[1].x = 1.0;
  points[1].y = 4.0;
  points[1].z = 3.0;

  std::vector<geometry_msgs::msg::Point> ans =
    math::geometry::transformPoints(pose, sensor_pose, points);
  auto ans0 = geometry_msgs::msg::Point();
  ans0.x = 1.0;
  ans0.y = -3.0;
  ans0.z = 2.0;
  auto ans1 = geometry_msgs::msg::Point();
  ans1.x = 1.0;
  ans1.y = -3.0;
  ans1.z = 4.0;
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsNoSensorPose)
{
  auto pose = filled_pose();
  auto sensor_pose = geometry_msgs::msg::Pose();
  std::vector<geometry_msgs::msg::Point> points(2);
  points[0].x = 1.0;
  points[0].y = 2.0;
  points[0].z = 3.0;
  points[1].x = 1.0;
  points[1].y = 4.0;
  points[1].z = 3.0;

  std::vector<geometry_msgs::msg::Point> ans =
    math::geometry::transformPoints(pose, sensor_pose, points);
  auto ans0 = geometry_msgs::msg::Point();
  ans0.x = 2.0;
  ans0.y = -1.0;
  ans0.z = 5.0;
  auto ans1 = geometry_msgs::msg::Point();
  ans1.x = 2.0;
  ans1.y = -1.0;
  ans1.z = 7.0;
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsEmptyVectorWithSensorPose)
{
  auto pose = filled_pose();
  auto sensor_pose = filled_pose();
  std::vector<geometry_msgs::msg::Point> points;

  std::vector<geometry_msgs::msg::Point> ans;
  EXPECT_NO_THROW(ans = math::geometry::transformPoints(pose, sensor_pose, points));
  EXPECT_EQ(ans.size(), size_t(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
