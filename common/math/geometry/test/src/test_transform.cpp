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

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/transform.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-3;

geometry_msgs::msg::Pose getFilledPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position = makePoint(1.0, 2.0, 3.0);
  pose.orientation =
    math::geometry::convertEulerAngleToQuaternion(makeVector(90.0 * M_PI / 180.0, 0.0));
  return pose;
}

TEST(Transform, getRelativePoseDifferent)
{
  geometry_msgs::msg::Pose pose0 = getFilledPose();
  geometry_msgs::msg::Pose pose1 = makePose(3.0, 0.0);

  geometry_msgs::msg::Pose ans = makePose(
    2.0, -3.0, 2.0,
    math::geometry::convertEulerAngleToQuaternion(makeVector(-90.0 * M_PI / 180.0, 0.0)));
  EXPECT_POSE_NEAR(math::geometry::getRelativePose(pose0, pose1), ans, EPS);
}

TEST(Transform, getRelativePoseIdentical)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  EXPECT_POSE_EQ(math::geometry::getRelativePose(pose, pose), geometry_msgs::msg::Pose());
}

TEST(Transform, transformPointRealPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);

  geometry_msgs::msg::Point ans = makePoint(2.0, -1.0, 5.0);
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, point), ans);
}

TEST(Transform, transformPointNoPose)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, point), point);
}

TEST(Transform, transformPointRealSensorPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Pose sensor_pose = getFilledPose();
  sensor_pose.orientation = geometry_msgs::msg::Quaternion();
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);

  geometry_msgs::msg::Point ans = makePoint(1.0, -3.0, 2.0);
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, sensor_pose, point), ans);
}

TEST(Transform, transformPointNoSensorPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Pose sensor_pose;
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);

  geometry_msgs::msg::Point ans = makePoint(2.0, -1.0, 5.0);
  EXPECT_POINT_EQ(math::geometry::transformPoint(pose, sensor_pose, point), ans);
}

TEST(Transform, transformPointsRealPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  std::vector<geometry_msgs::msg::Point> points{makePoint(1.0, 2.0, 3.0), makePoint(1.0, 4.0, 3.0)};

  geometry_msgs::msg::Point ans0 = makePoint(2.0, -1.0, 5.0);
  geometry_msgs::msg::Point ans1 = makePoint(2.0, -1.0, 7.0);

  std::vector<geometry_msgs::msg::Point> ans = math::geometry::transformPoints(pose, points);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsNoPose)
{
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Point> points{makePoint(1.0, 2.0, 3.0), makePoint(1.0, 4.0, 3.0)};

  std::vector<geometry_msgs::msg::Point> ans = math::geometry::transformPoints(pose, points);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], points[0], EPS);
  EXPECT_POINT_NEAR(ans[1], points[1], EPS);
}

TEST(Transform, transformPointsEmptyVector)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  std::vector<geometry_msgs::msg::Point> points;

  std::vector<geometry_msgs::msg::Point> ans;
  EXPECT_NO_THROW(ans = math::geometry::transformPoints(pose, points));
  EXPECT_EQ(ans.size(), size_t(0));
}

TEST(Transform, transformPointsRealSensorPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Pose sensor_pose = getFilledPose();
  sensor_pose.orientation = geometry_msgs::msg::Quaternion();
  std::vector<geometry_msgs::msg::Point> points{makePoint(1.0, 2.0, 3.0), makePoint(1.0, 4.0, 3.0)};

  std::vector<geometry_msgs::msg::Point> ans =
    math::geometry::transformPoints(pose, sensor_pose, points);
  geometry_msgs::msg::Point ans0 = makePoint(1.0, -3.0, 2.0);
  geometry_msgs::msg::Point ans1 = makePoint(1.0, -3.0, 4.0);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsNoSensorPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Pose sensor_pose;
  std::vector<geometry_msgs::msg::Point> points{makePoint(1.0, 2.0, 3.0), makePoint(1.0, 4.0, 3.0)};

  std::vector<geometry_msgs::msg::Point> ans =
    math::geometry::transformPoints(pose, sensor_pose, points);
  geometry_msgs::msg::Point ans0 = makePoint(2.0, -1.0, 5.0);
  geometry_msgs::msg::Point ans1 = makePoint(2.0, -1.0, 7.0);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_NEAR(ans[0], ans0, EPS);
  EXPECT_POINT_NEAR(ans[1], ans1, EPS);
}

TEST(Transform, transformPointsEmptyVectorWithSensorPose)
{
  geometry_msgs::msg::Pose pose = getFilledPose();
  geometry_msgs::msg::Pose sensor_pose = getFilledPose();
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
