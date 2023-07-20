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

/// @brief Testing the `CatmullRomSpline::getCollisionPointIn2D` function works valid.
/// In this test case, number of the control points of the catmull-rom spline (variable name `spline`) is 2, so the shape of the value `spline` is line segment.
TEST(CatmullRomSpline, GetCollisionWith2ControlPoints)
{
  /// @note The `spline` has control points p0 and p1. Control point p0 is point (x,y,z) = (0,0,0) and control point p1 is point (x,y,z) = (1,0,0) in the cartesian coordinate system.
  // [Snippet_construct_spline]
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  auto points = {p0, p1};
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 1);
  // [Snippet_construct_spline]
  /// @snippet test/test_catmull_rom_spline.cpp Snippet_construct_spline

  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find the collision point with `spline` and a line segment with start point (x,y,z) = (0,1,0) and end point (x,y,z) = (0,-1,0).
    /// `collision_s` variable in the test case is the denormalized s value in frenet coordinate along the `spline` curve.
    /// The collision point is in (x,y,z) = (0,0,0) in the cartesian coordinate system and the point is in `s=0.0`,
    /// So the return value of the `CatmullRomSpline::getCollisionPointIn2D` function (variable name `collision_s`) should be std::optional<double>(0.0)
    // [Snippet_getCollisionPointIn2D_with_0_1_0_0_-1_0]
    const auto collision_s = spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0)});
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.0);
    }
    // [Snippet_getCollisionPointIn2D_with_0_1_0_0_-1_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_getCollisionPointIn2D_with_0_1_0_0_-1_0
  }

  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find the collision point with `spline` and a line segment with start point (x,y,z) = (1,1,0) and end point (x,y,z) = (-1,-1,0).
    /// `collision_s` variable in the test case is the denormalized s value in frenet coordinate along the `spline` curve.
    /// The collision point is in (x,y,z) = (0,0,0) in the cartesian coordinate system and the point is in `s=0.0`,
    /// So the return value of the `CatmullRomSpline::getCollisionPointIn2D` function (variable name `collision_s`) should be std::optional<double>(0.0)
    // [Snippet_getCollisionPointIn2D_with_1_1_0_-1_-1_0]
    const auto collision_s = spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(-1).y(-1).z(0)});
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.0);
    }
    // [Snippet_getCollisionPointIn2D_with_1_1_0_-1_-1_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_getCollisionPointIn2D_with_1_1_0_-1_-1_0
  }

  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find the collision point with `spline` and a line segment with start point (x,y,z) = (1,1,0) and end point (x,y,z) = (0,-1,0).
    /// `collision_s` variable in the test case is the denormalized s value in frenet coordinate along the `spline` curve.
    /// The collision point is in (x,y,z) = (0,0.5,0) in the cartesian coordinate system and the point is in `s=0.5`,
    /// So the return value of the `CatmullRomSpline::getCollisionPointIn2D` function (variable name `collision_s`) should be std::optional<double>(0.5)
    // [Snippet_getCollisionPointIn2D_with_1_1_0_0_-1_0]
    const auto collision_s = spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0)});
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.5);
    }
    // [Snippet_getCollisionPointIn2D_with_1_1_0_0_-1_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_getCollisionPointIn2D_with_1_1_0_0_-1_0
  }

  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find that the `spline` and a line segment with start point (x,y,z) = (0,1,0) and end point (x,y,z) = (0,0.2,0) does not collide.
    /// If `CatmullRomSpline::getCollisionPointIn2D` function works expected, it returns std::nullopt;
    // [Snippet_getCollisionPointIn2D_with_0_1_0_0_02_0]
    const auto collision_s = spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0.2).z(0)});
    EXPECT_FALSE(collision_s);
    // [Snippet_getCollisionPointIn2D_with_0_1_0_0_02_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_getCollisionPointIn2D_with_0_1_0_0_02_0
  }
}

/// @brief Testing the `CatmullRomSpline::getCollisionPointIn2D` function works valid
/// In this test case, number of the control points of the catmull-rom spline (variable name `spline`) is 1, so the shape of the value `spline` is single point.
TEST(CatmullRomSpline, GetCollisionWith1ControlPoint)
{
  /// @note The variable `spline` has control point with point (x,y,z) = (0,1,0) in the cartesian coordinate system. So, `spline` is same as point (x,y,z) = (0,1,0).
  auto spline = math::geometry::CatmullRomSpline(
    {geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0)});
  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find the collision point with `spline` and a line segment with start point (x,y,z) = (0,1,0) and end point (x,y,z) = (0,-1,0).
    /// `collision_s` variable in the test case is the denormalized s value in frenet coordinate along the `spline` curve.
    /// The collision point is in (x,y,z) = (0,0,0) in the cartesian coordinate system and the point is in `s=0.0`,
    /// So the return value of the `CatmullRomSpline::getCollisionPointIn2D` function (variable name `collision_s`) should be std::optional<double>(0.0)
    // [Snippet_GetCollisionWith1ControlPoint_with_0_1_0_0_-1_0]
    const auto collision_s = spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0)});
    EXPECT_TRUE(collision_s);
    if (collision_s) {
      EXPECT_DOUBLE_EQ(collision_s.value(), 0.0);
    }
    // [Snippet_GetCollisionWith1ControlPoint_with_0_1_0_0_-1_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_GetCollisionWith1ControlPoint_with_0_1_0_0_-1_0
  }

  {
    /// @note Testing the `CatmullRomSpline::getCollisionPointIn2D` function can find that the `spline` and a line segment with start point (x,y,z) = (1,1,0) and end point (x,y,z) = (1,-1,0) does not collide.
    /// If `CatmullRomSpline::getCollisionPointIn2D` function works expected, it returns std::nullopt;
    // [Snippet_GetCollisionWith1ControlPoint_with_1_1_0_1_-1_0]
    EXPECT_FALSE(spline.getCollisionPointIn2D(
      {geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(0),
       geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(-1).z(0)}));
    // [Snippet_GetCollisionWith1ControlPoint_with_1_1_0_1_-1_0]
    /// @snippet test/test_catmull_rom_spline.cpp Snippet_GetCollisionWith1ControlPoint_with_1_1_0_1_-1_0
  }
}

TEST(CatmullRomSpline, GetCollisionPointIn2D)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  auto points = {p0, p1, p2};
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 2);
  geometry_msgs::msg::Point start;
  start.x = 0.1;
  start.y = 1.0;
  geometry_msgs::msg::Point goal;
  goal.x = 0.1;
  goal.y = -1.0;
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
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  p1.y = 3;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  p2.y = 6;
  auto points = {p0, p1, p2};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getMaximum2DCurvature(), 0);
}

TEST(CatmullRomSpline, Interpolate3Points)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  p1.y = 3;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  p2.y = 5;
  geometry_msgs::msg::Point p3;
  p3.x = 4;
  p3.y = 6;
  auto points = {p0, p1, p2, p3};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
}

TEST(CatmullRomSpline, Interpolate4Points)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  p1.y = 3;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  p2.y = 5;
  geometry_msgs::msg::Point p3;
  p3.x = 4;
  p3.y = 6;
  geometry_msgs::msg::Point p4;
  p4.x = 4;
  p4.y = 10;
  auto points = {p0, p1, p2, p3, p4};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
}

TEST(CatmullRomSpline, GetPoint)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  geometry_msgs::msg::Point p3;
  p3.x = 3;
  geometry_msgs::msg::Point p4;
  p4.x = 4;
  auto points = {p0, p1, p2, p3, p4};
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 4);
  auto point = spline.getPoint(3);
  EXPECT_DOUBLE_EQ(point.x, 3);
  EXPECT_DOUBLE_EQ(point.y, 0);
  EXPECT_DOUBLE_EQ(point.z, 0);
}

TEST(CatmullRomSpline, GetSValue)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  geometry_msgs::msg::Point p3;
  p3.x = 4;
  auto points = {p0, p1, p2, p3};
  auto spline = math::geometry::CatmullRomSpline(points);
  geometry_msgs::msg::Pose p;
  p.position.x = 0.1;
  p.position.y = 0;
  p.position.z = 0;
  const auto result = spline.getSValue(p);
  EXPECT_TRUE(result);
  EXPECT_TRUE(result.value() > 0.099);
  EXPECT_TRUE(result.value() < 0.101);
  p.position.x = 10;
  p.position.y = 0;
  p.position.z = 0;
  EXPECT_FALSE(spline.getSValue(p, 3));
}

TEST(CatmullRomSpline, GetSValue2)
{
  geometry_msgs::msg::Point p0;
  p0.x = 89122.2;
  p0.y = 43364.1;
  p0.z = 3.13364;
  geometry_msgs::msg::Point p1;
  p1.x = 89122.5;
  p1.y = 43363.8;
  p1.z = 3.13364;
  geometry_msgs::msg::Point p2;
  p2.x = 89122.8;
  p2.y = 43363.4;
  p2.z = 3.13364;
  geometry_msgs::msg::Point p3;
  p3.x = 89123.1;
  p3.y = 43363.0;
  p3.z = 3.13364;
  geometry_msgs::msg::Point p4;
  p4.x = 89123.4;
  p4.y = 43362.6;
  p4.z = 3.13364;
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
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  geometry_msgs::msg::Point p3;
  p3.x = 3;
  auto points = {p0, p1, p2, p3};
  auto spline = math::geometry::CatmullRomSpline(points);
  auto trajectory = spline.getTrajectory(0, 3, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_DECIMAL_EQ(trajectory[0].x, 0, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[1].x, 1, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[2].x, 2, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[3].x, 3, 0.00001);
  trajectory = spline.getTrajectory(3, 0, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_DECIMAL_EQ(trajectory[0].x, 3, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[1].x, 2, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[2].x, 1, 0.00001);
  EXPECT_DECIMAL_EQ(trajectory[3].x, 0, 0.00001);
}

TEST(CatmullRomSpline, CheckThrowingErrorWhenTheControlPointsAreNotEnough)
{
  EXPECT_THROW(
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>(0)),
    common::SemanticError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
