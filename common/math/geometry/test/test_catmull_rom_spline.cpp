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

#include <geometry/spline/catmull_rom_spline.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-3;

TEST(CatmullRomSpline, GetCollisionPointIn2D)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 0);
  geometry_msgs::msg::Point p2 = makePoint(2, 0);
  auto points = {p0, p1, p2};
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 2);
  geometry_msgs::msg::Point start = makePoint(0.1, 1);
  geometry_msgs::msg::Point goal = makePoint(0.1, -1);
  auto collision_s = spline.getCollisionPointIn2D(start, goal, false);
  EXPECT_TRUE(collision_s);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.value(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D({start, goal}, false);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.value(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D(start, goal, true);
  EXPECT_TRUE(collision_s);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.value(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D({start, goal}, true);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.value(), 0.1);
  }
}

TEST(CatmullRomSpline, Maximum2DCurvature)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 3);
  geometry_msgs::msg::Point p2 = makePoint(2, 6);
  auto points = {p0, p1, p2};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getMaximum2DCurvature(), 0);
}

TEST(CatmullRomSpline, Interpolate3Points)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 3);
  geometry_msgs::msg::Point p2 = makePoint(2, 5);
  geometry_msgs::msg::Point p3 = makePoint(4, 6);
  auto points = {p0, p1, p2, p3};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
}

TEST(CatmullRomSpline, Interpolate4Points)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 3);
  geometry_msgs::msg::Point p2 = makePoint(2, 5);
  geometry_msgs::msg::Point p3 = makePoint(4, 6);
  geometry_msgs::msg::Point p4 = makePoint(4, 10);
  auto points = {p0, p1, p2, p3, p4};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
}

TEST(CatmullRomSpline, GetPoint)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 0);
  geometry_msgs::msg::Point p2 = makePoint(2, 0);
  geometry_msgs::msg::Point p3 = makePoint(3, 0);
  geometry_msgs::msg::Point p4 = makePoint(4, 0);
  auto points = {p0, p1, p2, p3, p4};
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 4);
  auto point = spline.getPoint(3);
  EXPECT_POINT_EQ(point, makePoint(3, 0));
}

TEST(CatmullRomSpline, GetSValue)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 0);
  geometry_msgs::msg::Point p2 = makePoint(2, 0);
  geometry_msgs::msg::Point p3 = makePoint(4, 0);
  auto points = {p0, p1, p2, p3};
  auto spline = math::geometry::CatmullRomSpline(points);
  geometry_msgs::msg::Pose p = makePose(0.1, 0);
  const auto result = spline.getSValue(p);
  EXPECT_TRUE(result);
  EXPECT_NEAR(result.value(), 0.1, 0.001);
  p = makePose(10, 0);
  EXPECT_FALSE(spline.getSValue(p, 3));
}

TEST(CatmullRomSpline, GetSValue2)
{
  geometry_msgs::msg::Point p0 = makePoint(89122.2, 43364.1, 3.13364);
  geometry_msgs::msg::Point p1 = makePoint(89122.5, 43363.8, 3.13364);
  geometry_msgs::msg::Point p2 = makePoint(89122.8, 43363.4, 3.13364);
  geometry_msgs::msg::Point p3 = makePoint(89123.1, 43363.0, 3.13364);
  geometry_msgs::msg::Point p4 = makePoint(89123.4, 43362.6, 3.13364);
  auto points = {p0, p1, p2, p3, p4};
  auto spline = math::geometry::CatmullRomSpline(points);
  geometry_msgs::msg::Pose p;
  p.position.x = 89122.8;
  p.position.y = 43363.4;
  p.position.z = 3.13364;
  p.orientation.x = -0.0159808;
  p.orientation.y = -0.00566353;
  p.orientation.z = -0.453507;
  p.orientation.w = 0.891092;
  {
    const auto result = spline.getSValue(p);
    EXPECT_TRUE(result);
    if (result) {
      EXPECT_DOUBLE_EQ(result.value(), 0.92433178422155371);
    }
  }
  p.position.x = 89122.5;
  p.position.y = 43363.8;
  p.position.z = 3.13364;
  p.orientation.x = 0.0159365;
  p.orientation.y = -0.00578704;
  p.orientation.z = -0.446597;
  p.orientation.w = 0.894575;
  {
    const auto result = spline.getSValue(p);
    EXPECT_TRUE(result);
    if (result) {
      EXPECT_DOUBLE_EQ(result.value(), 0.42440442127906564);
    }
  }
}

TEST(CatmullRomSpline, GetTrajectory)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1 = makePoint(1, 0);
  geometry_msgs::msg::Point p2 = makePoint(2, 0);
  geometry_msgs::msg::Point p3 = makePoint(3, 0);
  auto points = {p0, p1, p2, p3};
  auto spline = math::geometry::CatmullRomSpline(points);
  auto trajectory = spline.getTrajectory(0, 3, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_NEAR(trajectory[0].x, 0, EPS);
  EXPECT_NEAR(trajectory[1].x, 1, EPS);
  EXPECT_NEAR(trajectory[2].x, 2, EPS);
  EXPECT_NEAR(trajectory[3].x, 3, EPS);
  trajectory = spline.getTrajectory(3, 0, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_NEAR(trajectory[0].x, 3, EPS);
  EXPECT_NEAR(trajectory[1].x, 2, EPS);
  EXPECT_NEAR(trajectory[2].x, 1, EPS);
  EXPECT_NEAR(trajectory[3].x, 0, EPS);
}

TEST(CatmullRomSpline, CheckThrowingErrorWhenTheControlPointsAreNotEnough)
{
  EXPECT_THROW(
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>(0)),
    common::SemanticError);
  EXPECT_THROW(
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>(1)),
    common::SemanticError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
