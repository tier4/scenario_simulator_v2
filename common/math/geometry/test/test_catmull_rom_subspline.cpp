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

#include <geometry/spline/catmull_rom_subspline.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"

constexpr double EPS = 1e-6;

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/// @brief Helper function generating line: p(0,0)-> p(1,3) -> p(2,6)
std::shared_ptr<math::geometry::CatmullRomSpline> makeLine()
{
  std::vector<geometry_msgs::msg::Point> points(3);
  points[1] = makePoint(1, 3);
  points[2] = makePoint(2, 6);
  return std::make_shared<math::geometry::CatmullRomSpline>(points);
}

TEST(CatmullRomSubspline, getLength)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline0(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(1.5, 4.5));
  EXPECT_DOUBLE_EQ(spline0.getLength(), std::hypot(1.0, 3.0));

  math::geometry::CatmullRomSubspline spline1(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(2.0, 6.0));
  EXPECT_DOUBLE_EQ(spline1.getLength(), std::hypot(1.5, 4.5));
}

TEST(CatmullRomSubspline, getLength_zero)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline0(spline_ptr, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(spline0.getLength(), 0.0);

  math::geometry::CatmullRomSubspline spline1(spline_ptr, 5.0, 5.0);
  EXPECT_DOUBLE_EQ(spline1.getLength(), 0.0);
}

/**
 * This test tests function that does not function properly yet. I have opened a github issue about
 * the spline collision calculation (which directly affects subspline collision calculation).
 */
TEST(CatmullRomSubspline, getCollisionPointIn2D)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline(
    spline_ptr, std::hypot(0.0, 0.0), std::hypot(1.5, 4.5));

  /// @brief Collision at the beginning of the spline
  std::vector<geometry_msgs::msg::Point> polygon0(4);
  polygon0[0] = makePoint(1.0, 1.0);
  polygon0[1] = makePoint(1.0, -1.0);
  polygon0[2] = makePoint(-1.0, -1.0);
  polygon0[3] = makePoint(-1.0, 1.0);

  const auto ans00 = spline.getCollisionPointIn2D(polygon0);
  EXPECT_TRUE(ans00);
  EXPECT_NEAR(ans00.value(), std::hypot(1.0 / 3.0, 1.0), EPS);
  const auto ans01 = spline.getCollisionPointIn2D(polygon0, true);
  EXPECT_TRUE(ans01);
  EXPECT_NEAR(ans01.value(), std::hypot(1.0 / 3.0, 1.0), EPS);
  const auto ans02 = spline.getCollisionPointIn2D(polygon0, false, false);
  EXPECT_FALSE(ans02);
  const auto ans03 = spline.getCollisionPointIn2D(polygon0, true, false);
  EXPECT_FALSE(ans03);

  /// @brief Collision at the end of the spline
  std::vector<geometry_msgs::msg::Point> polygon1(3);
  polygon1[0] = makePoint(0.0, 5.0);
  polygon1[1] = makePoint(2.0, 5.0);
  polygon1[2] = makePoint(2.0, 3.0);

  const auto ans10 = spline.getCollisionPointIn2D(polygon1);
  EXPECT_TRUE(ans10);
  EXPECT_NEAR(ans10.value(), std::hypot(1.25, 3.75), EPS);
  const auto ans11 = spline.getCollisionPointIn2D(polygon1, true);
  EXPECT_TRUE(ans11);
  EXPECT_NEAR(ans11.value(), std::hypot(1.25, 3.75), EPS);
  const auto ans12 = spline.getCollisionPointIn2D(polygon1, false, false);
  EXPECT_FALSE(ans12);
  const auto ans13 = spline.getCollisionPointIn2D(polygon1, true, false);
  EXPECT_FALSE(ans13);
}

/**
 * This test fails because the subspline logic is incorrect. It obtains the collision with
 * the polygon (either first or lst one depending on the search_backward parameter) and then - if
 * the collision point is outside of subspline - returns nothing. This leads to incorrect behavior
 * when the polygon collides with the spline in more than one point, but some of these points are 
 * not on the subspline. If the polygon collides with the spline 2 times and the first collision 
 * point is outside of the subspline, but the second collision point is inside of the subspline and
 * the search_backward parameter is set to true, the second (valid) collision is ignored and nothing
 * is returned.
 * The possible fix for this may be to add function in spline that returns all collision points, not
 * only the first one, and in subspline check all of them and choose the first one that is in the
 * subspline.
 */
TEST(CatmullRomSubspline, getCollisionPointIn2D_shiftedBeginning)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(1.0, 3.0));

  /// @brief Collision at the beginning of the spline
  std::vector<geometry_msgs::msg::Point> polygon0(3);
  polygon0[0] = makePoint(0.0, 3.0);
  polygon0[1] = makePoint(0.0, 0.0);
  polygon0[2] = makePoint(3.0, 0.0);

  const auto ans00 = spline.getCollisionPointIn2D(polygon0);
  EXPECT_TRUE(ans00);
  EXPECT_NEAR(ans00.value(), std::hypot(0.75 - 0.5, 2.25 - 1.5), EPS);
  const auto ans01 = spline.getCollisionPointIn2D(polygon0, true);
  EXPECT_TRUE(ans01);
  EXPECT_NEAR(ans01.value(), std::hypot(0.75 - 0.5, 2.25 - 1.5), EPS);
  const auto ans02 = spline.getCollisionPointIn2D(polygon0, false, false);
  EXPECT_FALSE(ans02);
  const auto ans03 = spline.getCollisionPointIn2D(polygon0, true, false);
  EXPECT_FALSE(ans03);

  /// @brief Collision at the end of the spline
  std::vector<geometry_msgs::msg::Point> polygon1(3);
  polygon1[0] = makePoint(2.0, 1.0);
  polygon1[1] = makePoint(2.0, 4.0);
  polygon1[2] = makePoint(-1.0, 4.0);

  const auto ans10 = spline.getCollisionPointIn2D(polygon1);
  EXPECT_TRUE(ans10);
  EXPECT_NEAR(ans10.value(), std::hypot(0.75 - 0.5, 2.25 - 1.5), EPS);
  const auto ans11 = spline.getCollisionPointIn2D(polygon1, true);
  EXPECT_TRUE(ans11);
  EXPECT_NEAR(ans11.value(), std::hypot(0.75 - 0.5, 2.25 - 1.5), EPS);
  const auto ans12 = spline.getCollisionPointIn2D(polygon1, false, false);
  EXPECT_FALSE(ans12);
  const auto ans13 = spline.getCollisionPointIn2D(polygon1, true, false);
  EXPECT_FALSE(ans13);
}

TEST(CatmullRomSubspline, getCollisionPointIn2D_edge)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(1.5, 4.5));

  std::vector<geometry_msgs::msg::Point> polygon(4);
  polygon[0] = makePoint(-2, -2);
  polygon[1] = makePoint(-2, -1);
  polygon[2] = makePoint(-1, -1);
  polygon[3] = makePoint(-1, -2);

  const auto ans0 = spline.getCollisionPointIn2D(polygon);
  EXPECT_FALSE(ans0);
  const auto ans1 = spline.getCollisionPointIn2D(polygon, true);
  EXPECT_FALSE(ans1);
  const auto ans2 = spline.getCollisionPointIn2D(polygon, false, false);
  EXPECT_FALSE(ans2);
  const auto ans3 = spline.getCollisionPointIn2D(polygon, true, false);
  EXPECT_FALSE(ans3);
}

TEST(CatmullRomSubspline, getCollisionPointIn2D_base)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(1.5, 4.5));

  std::vector<geometry_msgs::msg::Point> polygon(4);
  polygon[0] = makePoint(0, 0);
  polygon[1] = makePoint(0, 1);
  polygon[2] = makePoint(1, 1);
  polygon[3] = makePoint(1, 0);

  const auto ans0 = spline.getCollisionPointIn2D(polygon);
  EXPECT_FALSE(ans0);
  const auto ans1 = spline.getCollisionPointIn2D(polygon, true);
  EXPECT_FALSE(ans1);
  const auto ans2 = spline.getCollisionPointIn2D(polygon, false, false);
  EXPECT_FALSE(ans2);
  const auto ans3 = spline.getCollisionPointIn2D(polygon, true, false);
  EXPECT_FALSE(ans3);
}

TEST(CatmullRomSubspline, getCollisionPointIn2D_wrongPolygon)
{
  const auto spline_ptr = makeLine();

  math::geometry::CatmullRomSubspline spline(
    spline_ptr, std::hypot(0.5, 1.5), std::hypot(1.5, 4.5));

  std::vector<geometry_msgs::msg::Point> polygon0;

  const auto ans00 = spline.getCollisionPointIn2D(polygon0);
  EXPECT_FALSE(ans00);
  const auto ans01 = spline.getCollisionPointIn2D(polygon0, true);
  EXPECT_FALSE(ans01);
  const auto ans02 = spline.getCollisionPointIn2D(polygon0, false, false);
  EXPECT_FALSE(ans02);
  const auto ans03 = spline.getCollisionPointIn2D(polygon0, true, false);
  EXPECT_FALSE(ans03);

  std::vector<geometry_msgs::msg::Point> polygon1(1);

  const auto ans10 = spline.getCollisionPointIn2D(polygon1);
  EXPECT_FALSE(ans10);
  const auto ans11 = spline.getCollisionPointIn2D(polygon1, true);
  EXPECT_FALSE(ans11);
  const auto ans12 = spline.getCollisionPointIn2D(polygon1, false, false);
  EXPECT_FALSE(ans12);
  const auto ans13 = spline.getCollisionPointIn2D(polygon1, true, false);
  EXPECT_FALSE(ans13);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
