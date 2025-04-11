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
#include <geometry/spline/catmull_rom_spline.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

constexpr double EPS = 1e-6;

/// @brief Helper function generating line: p(0,0)-> p(1,3) -> p(2,6)
math::geometry::CatmullRomSpline makeLine()
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 3.0), makePoint(2.0, 6.0)};
  return math::geometry::CatmullRomSpline(points);
}

/// @brief Helper function generating sloped line: p(0,0,0)-> p(1,3,1) -> p(2,6,2)
math::geometry::CatmullRomSpline makeSlopedLine()
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0, 0.0), makePoint(1.0, 3.0, 1.0), makePoint(2.0, 6.0, 2.0)};
  return math::geometry::CatmullRomSpline(points);
}

/// @brief Helper function generating curve: p(0,0)-> p(1,1)-> p(2,0)
math::geometry::CatmullRomSpline makeCurve()
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 1.0), makePoint(2.0, 0.0)};
  return math::geometry::CatmullRomSpline(points);
}

/// @brief Helper function generating curve: p(0,0)-> p(1,1)-> p(0,2)
math::geometry::CatmullRomSpline makeCurve2()
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 1.0), makePoint(0.0, 2.0)};
  return math::geometry::CatmullRomSpline(points);
}

/**
 * Add an offset to the given point in a specified direction.
 *
 * @param point The point to which the offset will be added.
 * @param offset The value of the offset.
 * @param theta The angle in radians representing the direction.
 */
void addOffset(geometry_msgs::msg::Point & point, const double offset, const double theta)
{
  point.x += std::cos(theta) * offset;
  point.y += std::sin(theta) * offset;
}

/// @brief Testing the `CatmullRomSpline::getCollisionPointIn2D` function works valid.
/// In this test case, number of the control points of the catmull-rom spline (variable name `spline`) is 2, so the shape of the value `spline` is line segment.
TEST(CatmullRomSpline, getCollisionPointIn2D_2ControlPoints)
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
TEST(CatmullRomSpline, getCollisionPointIn2D_1ControlPoint)
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
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(2.0, 0.0)};
  const math::geometry::CatmullRomSpline spline(points);

  EXPECT_NEAR(spline.getLength(), 2.0, EPS);
  geometry_msgs::msg::Point start = makePoint(0.1, 1.0), goal = makePoint(0.1, -1.0);

  const auto collision_s0 = spline.getCollisionPointIn2D(start, goal, false);
  EXPECT_TRUE(collision_s0);
  if (collision_s0) {
    EXPECT_NEAR(collision_s0.value(), 0.1, EPS);
  }
  const auto collision_s1 = spline.getCollisionPointIn2D({start, goal}, false);
  EXPECT_TRUE(collision_s1);
  if (collision_s1) {
    EXPECT_NEAR(collision_s1.value(), 0.1, EPS);
  }
  const auto collision_s2 = spline.getCollisionPointIn2D(start, goal, true);
  EXPECT_TRUE(collision_s2);
  if (collision_s2) {
    EXPECT_NEAR(collision_s2.value(), 0.1, EPS);
  }
  const auto collision_s3 = spline.getCollisionPointIn2D({start, goal}, true);
  EXPECT_TRUE(collision_s3);
  if (collision_s3) {
    EXPECT_NEAR(collision_s3.value(), 0.1, EPS);
  }
}

TEST(CatmullRomSpline, getCollisionPointIn2D)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(2.0, 0.0), makePoint(4.0, 0.0)};
  const math::geometry::CatmullRomSpline spline(points);

  const geometry_msgs::msg::Point start0 = makePoint(0.1, 1.0), goal0 = makePoint(0.1, -1.0);

  const auto collision_s00 = spline.getCollisionPointIn2D(start0, goal0, false);
  EXPECT_TRUE(collision_s00);
  EXPECT_NEAR(collision_s00.value(), 0.1, EPS);

  const auto collision_s01 = spline.getCollisionPointIn2D({start0, goal0}, false);
  EXPECT_TRUE(collision_s01);
  EXPECT_NEAR(collision_s01.value(), 0.1, EPS);

  const auto collision_s02 = spline.getCollisionPointIn2D(start0, goal0, true);
  EXPECT_TRUE(collision_s02);
  EXPECT_NEAR(collision_s02.value(), 0.1, EPS);

  const auto collision_s03 = spline.getCollisionPointIn2D({start0, goal0}, true);
  EXPECT_TRUE(collision_s03);
  EXPECT_NEAR(collision_s03.value(), 0.1, EPS);

  const geometry_msgs::msg::Point start1 = makePoint(3.5, 1.0), goal1 = makePoint(3.5, -1.0);

  const auto collision_s10 = spline.getCollisionPointIn2D(start1, goal1, false);
  EXPECT_TRUE(collision_s10);
  EXPECT_NEAR(collision_s10.value(), 3.5, EPS);

  const auto collision_s11 = spline.getCollisionPointIn2D({start1, goal1}, false);
  EXPECT_TRUE(collision_s11);
  EXPECT_NEAR(collision_s11.value(), 3.5, EPS);

  const auto collision_s12 = spline.getCollisionPointIn2D(start1, goal1, true);
  EXPECT_TRUE(collision_s12);
  EXPECT_NEAR(collision_s12.value(), 3.5, EPS);

  const auto collision_s13 = spline.getCollisionPointIn2D({start1, goal1}, true);
  EXPECT_TRUE(collision_s13);
  EXPECT_NEAR(collision_s13.value(), 3.5, EPS);
}

TEST(CatmullRomSpline, getCollisionPointIn2DNoCollision)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();

  const geometry_msgs::msg::Point start = makePoint(0.5, 0.0), goal = makePoint(1.5, 0.0);

  const auto collision_s0 = spline.getCollisionPointIn2D(start, goal, false);
  EXPECT_FALSE(collision_s0);

  const auto collision_s1 = spline.getCollisionPointIn2D({start, goal}, false);
  EXPECT_FALSE(collision_s1);

  const auto collision_s2 = spline.getCollisionPointIn2D(start, goal, true);
  EXPECT_FALSE(collision_s2);

  const auto collision_s3 = spline.getCollisionPointIn2D({start, goal}, true);
  EXPECT_FALSE(collision_s3);
}

TEST(CatmullRomSpline, getCollisionPointIn2DPolygon)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();

  const std::vector<geometry_msgs::msg::Point> polygon{
    makePoint(0.0, 1.0), makePoint(-1.0, 0.0), makePoint(0.0, -1.0), makePoint(1.0, 0.0)};

  const auto collision_s0 = spline.getCollisionPointIn2D(polygon);
  EXPECT_TRUE(collision_s0);
  EXPECT_NEAR(collision_s0.value(), 0.56727227, EPS);

  const auto collision_s1 = spline.getCollisionPointIn2D(polygon, true);
  EXPECT_TRUE(collision_s1);
  EXPECT_NEAR(collision_s1.value(), 0.56727227, EPS);
}

TEST(CatmullRomSpline, getCollisionPointIn2DEmpty)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const std::vector<geometry_msgs::msg::Point> polygon;

  EXPECT_THROW(spline.getCollisionPointIn2D(polygon), common::SimulationError);
  EXPECT_THROW(spline.getCollisionPointIn2D(polygon, true), common::SimulationError);
}

TEST(CatmullRomSpline, getMaximum2DCurvatureLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  EXPECT_DOUBLE_EQ(spline.getMaximum2DCurvature(), 0.0);
}

TEST(CatmullRomSpline, getMaximum2DCurvatureCurve)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(0.5, 0.5), makePoint(1.0, 0.0), makePoint(2.0, -1.0),
    makePoint(3.0, 0.0)};
  const math::geometry::CatmullRomSpline spline(points);
  constexpr double eps = 0.1;
  EXPECT_NEAR(spline.getMaximum2DCurvature(), -6.0, eps);
}

TEST(CatmullRomSpline, getPolygon)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(2.0, 0.0), makePoint(3.0, 0.0),
    makePoint(4.0, 0.0)};
  math::geometry::CatmullRomSpline spline(points);
  std::vector<geometry_msgs::msg::Point> polygon = spline.getPolygon(1.0, 4);

  EXPECT_EQ(polygon.size(), static_cast<size_t>(24));
  // pair of triangles
  EXPECT_POINT_NEAR(polygon[0], makePoint(0.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[1], makePoint(0.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[2], makePoint(1.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[3], makePoint(0.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[4], makePoint(1.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[5], makePoint(1.0, 0.5), EPS);
  // pair of triangles
  EXPECT_POINT_NEAR(polygon[6], makePoint(1.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[7], makePoint(1.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[8], makePoint(2.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[9], makePoint(1.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[10], makePoint(2.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[11], makePoint(2.0, 0.5), EPS);
  // pair of triangles
  EXPECT_POINT_NEAR(polygon[12], makePoint(2.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[13], makePoint(2.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[14], makePoint(3.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[15], makePoint(2.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[16], makePoint(3.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[17], makePoint(3.0, 0.5), EPS);
  // pair of triangles
  EXPECT_POINT_NEAR(polygon[18], makePoint(3.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[19], makePoint(3.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[20], makePoint(4.0, 0.5), EPS);
  EXPECT_POINT_NEAR(polygon[21], makePoint(3.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[22], makePoint(4.0, -0.5), EPS);
  EXPECT_POINT_NEAR(polygon[23], makePoint(4.0, 0.5), EPS);
}

TEST(CatmullRomSpline, getPolygonEdge)
{
  math::geometry::CatmullRomSpline spline = makeCurve();
  std::vector<geometry_msgs::msg::Point> polygon0 = spline.getPolygon(1.0, 0);
  EXPECT_EQ(polygon0.size(), static_cast<size_t>(0));

  std::vector<geometry_msgs::msg::Point> polygon1 = spline.getPolygon(1.0, 1);
  EXPECT_EQ(polygon1.size(), static_cast<size_t>(6));

  std::vector<geometry_msgs::msg::Point> polygon2 = spline.getPolygon(1.0, 2);
  EXPECT_EQ(polygon2.size(), static_cast<size_t>(12));
}

TEST(CatmullRomSpline, initializationLine)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 3.0), makePoint(2.0, 6.0)};

  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
  auto spline = math::geometry::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getMaximum2DCurvature(), 0.0);
}

TEST(CatmullRomSpline, initializationCurve)
{
  const std::vector<geometry_msgs::msg::Point> points{

    makePoint(0.0, 0.0), makePoint(1.0, 3.0), makePoint(2.0, 5.0), makePoint(4.0, 6.0),
    makePoint(4.0, 1.0)};
  EXPECT_NO_THROW(auto spline = math::geometry::CatmullRomSpline(points));
}

TEST(CatmullRomSpline, getLengthLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  EXPECT_NEAR(spline.getLength(), std::hypot(2.0, 6.0), EPS);
}

TEST(CatmullRomSpline, getLengthCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  constexpr double eps = 0.1;
  EXPECT_NEAR(spline.getLength(), 3.0, eps);
}

TEST(CatmullRomSpline, getPointLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  auto point = spline.getPoint(std::hypot(1.5, 4.5));
  EXPECT_POINT_NEAR(point, makePoint(1.5, 4.5), EPS);
}

TEST(CatmullRomSpline, getPointCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto point = spline.getPoint(1.5);
  constexpr double eps = 0.15;
  EXPECT_POINT_NEAR(point, makePoint(1.0, 1.0), eps);
}

TEST(CatmullRomSpline, getTangentVectorLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  const auto vec = spline.getTangentVector(1.0);
  const double norm_vec = std::hypot(vec.x, vec.y, vec.z);
  const auto ans = makeVector(1.5, 4.5);
  const double norm_ans = std::hypot(ans.x, ans.y, ans.z);
  EXPECT_NEAR(vec.x / norm_vec, ans.x / norm_ans, EPS);
  EXPECT_NEAR(vec.y / norm_vec, ans.y / norm_ans, EPS);
  EXPECT_NEAR(vec.z / norm_vec, ans.z / norm_ans, EPS);
}

TEST(CatmullRomSpline, getTangentVectorCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto vec = spline.getTangentVector(1.5);
  const double norm_vec = std::hypot(vec.x, vec.y, vec.z);
  const auto ans = makeVector(1.0, 0.0);
  constexpr double eps = 0.15;
  EXPECT_NEAR(vec.x / norm_vec, ans.x, eps);
  EXPECT_NEAR(vec.y / norm_vec, ans.y, eps);
  EXPECT_NEAR(vec.z / norm_vec, ans.z, eps);
}

TEST(CatmullRomSpline, getNormalVectorLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  const auto vec = spline.getNormalVector(1.0);
  const double norm_vec = std::hypot(vec.x, vec.y, vec.z);
  const auto ans = makeVector(-4.5, 1.5);
  const double norm_ans = std::hypot(ans.x, ans.y, ans.z);
  EXPECT_NEAR(vec.x / norm_vec, ans.x / norm_ans, EPS);
  EXPECT_NEAR(vec.y / norm_vec, ans.y / norm_ans, EPS);
  EXPECT_NEAR(vec.z / norm_vec, ans.z / norm_ans, EPS);
}

TEST(CatmullRomSpline, getNormalVectorCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto vec = spline.getNormalVector(1.5);
  const double norm_vec = std::hypot(vec.x, vec.y, vec.z);
  const auto ans = makeVector(0.0, 1.0);
  constexpr double eps = 0.15;
  EXPECT_NEAR(vec.x / norm_vec, ans.x, eps);
  EXPECT_NEAR(vec.y / norm_vec, ans.y, eps);
  EXPECT_NEAR(vec.z / norm_vec, ans.z, eps);
}

TEST(CatmullRomSpline, getPoseLine)
{
  const math::geometry::CatmullRomSpline spline = makeLine();
  const auto pose = spline.getPose(std::hypot(1.5, 4.5));
  EXPECT_POINT_NEAR(pose.position, makePoint(1.5, 4.5), EPS);
  EXPECT_QUATERNION_NEAR(
    pose.orientation,
    math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, std::atan(3.0))), EPS);
}

TEST(CatmullRomSpline, getPoseLineWithPitch)
{
  const math::geometry::CatmullRomSpline spline = makeSlopedLine();
  const auto pose = spline.getPose(std::hypot(1.5, 4.5, 1.5), true);
  EXPECT_POINT_NEAR(pose.position, makePoint(1.5, 4.5, 1.5), EPS);
  EXPECT_QUATERNION_NEAR(
    pose.orientation,
    math::geometry::convertEulerAngleToQuaternion(
      makeVector(0.0, std::atan2(-1.0, std::sqrt(1.0 + 3.0 * 3.0)), std::atan(3.0))),
    EPS);
}

TEST(CatmullRomSpline, getPoseCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto pose = spline.getPose(1.5);
  constexpr double eps = 0.02;
  EXPECT_POINT_NEAR(pose.position, makePoint(1.0, 1.0), eps);
  EXPECT_QUATERNION_NEAR(pose.orientation, geometry_msgs::msg::Quaternion(), eps);
}

TEST(CatmullRomSpline, getPoseCurveWithPitch)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto pose = spline.getPose(1.5, true);
  constexpr double eps = 0.02;
  EXPECT_POINT_NEAR(pose.position, makePoint(1.0, 1.0), eps);
  EXPECT_QUATERNION_NEAR(pose.orientation, geometry_msgs::msg::Quaternion(), eps);
}

TEST(CatmullRomSpline, getSValue1)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(2.0, 0.0), makePoint(4.0, 0.0)};
  const auto spline = math::geometry::CatmullRomSpline(points);
  const auto result = spline.getSValue(makePose(0.1, 0.0));
  EXPECT_TRUE(result);
  EXPECT_NEAR(result.value(), 0.1, EPS);
  EXPECT_FALSE(spline.getSValue(makePose(10.0, 0.0), 3.0));
}

TEST(CatmullRomSpline, getSValue2)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(89122.2, 43364.1, 3.13364), makePoint(89122.5, 43363.8, 3.13364),
    makePoint(89122.8, 43363.4, 3.13364), makePoint(89123.1, 43363.0, 3.13364),
    makePoint(89123.4, 43362.6, 3.13364)};
  auto spline = math::geometry::CatmullRomSpline(points);

  geometry_msgs::msg::Pose pose0;
  pose0.position.x = 89122.8;
  pose0.position.y = 43363.4;
  pose0.position.z = 3.13364;
  pose0.orientation.x = -0.0159808;
  pose0.orientation.y = -0.00566353;
  pose0.orientation.z = -0.453507;
  pose0.orientation.w = 0.891092;
  const auto result0 = spline.getSValue(pose0);
  EXPECT_TRUE(result0);
  EXPECT_DOUBLE_EQ(result0.value(), 0.92433178422155371);

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 89122.5;
  pose1.position.y = 43363.8;
  pose1.position.z = 3.13364;
  pose1.orientation.x = 0.0159365;
  pose1.orientation.y = -0.00578704;
  pose1.orientation.z = -0.446597;
  pose1.orientation.w = 0.894575;
  const auto result1 = spline.getSValue(pose1);
  EXPECT_TRUE(result1);
  EXPECT_DOUBLE_EQ(result1.value(), 0.42440442127906564);
}

TEST(CatmullRomSpline, getSValueEdge)
{
  const math::geometry::CatmullRomSpline spline = makeCurve();
  const auto pose = makePose(-2.0, -2.0);
  EXPECT_FALSE(spline.getSValue(pose, 1.0));
}

TEST(CatmullRomSpline, getSquaredDistanceIn2D)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();
  const auto distance = spline.getSquaredDistanceIn2D(makePoint(2.0, 2.0), 1.47895776);
  EXPECT_NEAR(distance, 2.0, 1e-2);
}

TEST(CatmullRomSpline, getSquaredDistanceIn2DSamePoint)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();
  const auto distance = spline.getSquaredDistanceIn2D(makePoint(1.0, 1.0), 1.47895776);
  EXPECT_NEAR(distance, 0.0, 1e-2);
}

TEST(CatmullRomSpline, getSquaredDistanceVector)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();
  const auto vector = spline.getSquaredDistanceVector(makePoint(2.0, 2.0), 1.47895776);
  EXPECT_POINT_NEAR(vector, makeVector(1.0, 1.0), 1e-2);
}

TEST(CatmullRomSpline, getSquaredDistanceVectorSamePoint)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();
  const auto vector = spline.getSquaredDistanceVector(makePoint(1.0, 1.0), 1.47895776);
  EXPECT_POINT_NEAR(vector, makeVector(0.0, 0.0), 1e-2);
}

TEST(CatmullRomSpline, getTrajectoryLine)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(2.0, 0.0), makePoint(3.0, 0.0)};
  const auto spline = math::geometry::CatmullRomSpline(points);

  const auto trajectory = spline.getTrajectory(0.0, 3.0, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_POINT_NEAR(trajectory[0], makePoint(0.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory[1], makePoint(1.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory[2], makePoint(2.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory[3], makePoint(3.0, 0.0), EPS);

  const auto trajectory_reverse = spline.getTrajectory(3.0, 0.0, 1.0);
  EXPECT_EQ(trajectory_reverse.size(), static_cast<size_t>(4));
  EXPECT_POINT_NEAR(trajectory_reverse[0], makePoint(3.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory_reverse[1], makePoint(2.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory_reverse[2], makePoint(1.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory_reverse[3], makePoint(0.0, 0.0), EPS);
}

TEST(CatmullRomSpline, getTrajectoryCurve)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();

  const auto trajectory = spline.getTrajectory(0.0, 2.957916, 0.5);
  EXPECT_POINT_NEAR(trajectory[0], makePoint(0.0, 0.0), EPS);
  EXPECT_POINT_NEAR(trajectory[1], makePoint(0.559992, 0.336669), EPS);
  EXPECT_POINT_NEAR(trajectory[2], makePoint(0.893292, 0.673338), EPS);
  EXPECT_POINT_NEAR(trajectory[3], makePoint(0.999898, 1.010091), EPS);
  EXPECT_POINT_NEAR(trajectory[4], makePoint(0.87779, 1.349586), EPS);
  EXPECT_POINT_NEAR(trajectory[5], makePoint(0.525168, 1.68908), EPS);
  EXPECT_POINT_NEAR(trajectory[6], makePoint(0.0, 2.0), EPS);

  const auto trajectory_offset = spline.getTrajectory(0.0, 2.957916, 0.5, -1.0);
  EXPECT_POINT_NEAR(trajectory_offset[0], makePoint(0.447214, -0.894427), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[1], makePoint(1.161918, -0.461883), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[2], makePoint(1.730462, 0.126395), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[3], makePoint(1.999695, 1.030269), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[4], makePoint(1.697341, 1.922592), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[5], makePoint(1.112457, 2.498458), EPS);
  EXPECT_POINT_NEAR(trajectory_offset[6], makePoint(0.447213, 2.894428), EPS);
}

TEST(CatmullRomSpline, getTrajectoryEmpty)
{
  const math::geometry::CatmullRomSpline spline = makeCurve2();
  const auto trajectory = spline.getTrajectory(0.0, 1.47895776, 2.0);
  constexpr double eps = 1e-2;
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(2));
  EXPECT_POINT_NEAR(trajectory[0], makePoint(0.0, 0.0), eps);
  EXPECT_POINT_NEAR(trajectory[1], makePoint(1.0, 1.0), eps);
}

TEST(CatmullRomSpline, initializationNotEnoughControlPoints)
{
  const std::vector<geometry_msgs::msg::Point> points;
  EXPECT_THROW(math::geometry::CatmullRomSpline{points}, common::SemanticError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
