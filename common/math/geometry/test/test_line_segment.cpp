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

#include <cmath>
#include <geometry/polygon/line_segment.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"

TEST(LineSegmentTest, GetPoint)
{
  {
    /**
     * y
     * ^
     * |    + P1 = (1,1,1)
     * |   $
     * |  $
     * | $
     * |$
     * +-------> x
     * P0 = (0,0,0)
     * 
     * -----------------------------------------------------------
     * $$$$$$$$$ Line segment (start point is P0, end point is P1)
     */
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note if s = 0, the value should be start point.
    EXPECT_POINT_EQ(
      line.getPoint(0, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0));
    /// @note if s = 1, and denormalize_s = false, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(1, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note if s is not in range s = [0,1], getPoint function should be throw error.
    EXPECT_THROW(line.getPoint(2, false), common::SimulationError);
    EXPECT_THROW(line.getPoint(2, true), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, false), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, true), common::SimulationError);
    /// @note if s = std::hypot((0,0,0), (1,1,1)), and denormalize_s = true, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(std::sqrt(3), true),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    EXPECT_DOUBLE_EQ(line.getLength(), std::sqrt(3));
  }
}

TEST(LineSegment, GetPose)
{
  {
    /**
     * z      y
     * ^     /
     * + (0,0,1)
     * $   /
     * $  /
     * $ /
     * $/
     * +-------> x
     * P0 = (0,0,0)
     * 
     * -----------------------------------------------------------
     * $$$$$$$$$ Line segment (start point is P0, end point is P1)
     */
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(1));
    /// @note if s = 0, the value should be start point.
    EXPECT_POSE_EQ(
      line.getPose(0, false),
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)));
  }
}

TEST(LineSegment, isIntersect2D)
{
  {
    /**
     * y
     * ^
     * |
     * + (x,y,z) = (0,1,0)
     * $
     * $
     * $
     * +----------------------> x
     * $
     * $
     * $
     * + (x,y,z) = (0,-1,0)
     * 
     * -----------------------------------------------------------
     * $$$$$$$$$$$$$ Line segment
     */
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0));
    /// @note check intersection with point (0,0,0)
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    /// @note check intersection with point (0,0,-1)
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(-1)));
    /// @note check intersection with point (0,1,0)
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0)));
    /// @note check intersection with point (0,2,0)
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(2).z(0)));
    /// @note check intersection with point (0,-2,0)
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-2).z(0)));
  }
}

/// @brief In this test case, we check collision with the line segment with start point (x,y,z) = (0,-1,0) and end point (x,y,z) = (0,1,0) in the cartesian coordinate system. (variable name "line").
TEST(LineSegment, getIntersection2DSValue)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0));
    /**
     * @note Testing the `getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,0,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 0.5.
     * If so, the variable `collision_s` should be `std::optional<double>(0.5)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_0_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0), false);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 0.5);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_0_0

    /**
     * @note Testing the `getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,1,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 1.0.
     * If so, the variable `collision_s` should be `std::optional<double>(1.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_1_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0), false);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 1.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_1_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_1_0

    /**
     * @note Testing the `getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,1,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 1.0.
     * And, the 2nd argument of the `getIntersection2DSValue` (denoramalized_s) is true, so the return value should be 1.0 (noramalized s value.) * 2.0 (length of the `line`) = 2.0.
     * If so, the variable `collision_s` should be `std::optional<double>(2.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0), true);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 2.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized

    /**
     * @note Testing the `getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,0,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 0.5.
     * And, the 2nd argument of the `getIntersection2DSValue` (denoramalized_s) is true, so the return value should be 0.5 (noramalized s value.) * 2.0 (length of the `line`) = 1.0.
     * If so, the variable `collision_s` should be `std::optional<double>(1.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0), true);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 1.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized

    /**
     * @note Testing the `getIntersection2DSValue` function can find that the point with (x,y,z) = (1,0,0) in the cartesian coordinate system does not collide to `line.`.
     * If the function works valid, the variable `collision_s` should be `std::nullopt`.
     */
    // [Snippet_getIntersection2DSValue_with_point_1_0_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(0).z(0), false);
      EXPECT_FALSE(collision_s);
    }
    // [Snippet_getIntersection2DSValue_with_point_1_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_1_0_0

    /**
     * @note Testing the `getIntersection2D` function can detect erorrs getting intersection with exact same line segment.
     * In this case, any s value can be a intersection point, so we cannot return single value.
     */
    // [Snippet_getIntersection2D_with_line]
    {
      EXPECT_THROW(line.getIntersection2D(line), common::SimulationError);
    }
    // [Snippet_getIntersection2D_with_line]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_with_line


    /**
     * @note Testing the `getIntersection2D` function can detect erorrs getting intersection with part of the line segment `line`.
     * In this case, any s value can be a intersection point, so we cannot return single value.
     */
    // [Snippet_getIntersection2D_with_part_of_line]
    {
      EXPECT_THROW(
        line.getIntersection2D(math::geometry::LineSegment(
          geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
          geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))),
        common::SimulationError);
    }
    // [Snippet_getIntersection2D_with_part_of_line]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_with_part_of_line

    /// @note @note Testing the `getIntersection2D` function can find collision with the line segment with start point (x,y,z) = (1,0,0) and end point (x,y,z) = (-1,0,0) in the cartesian coordinate system and `line`.
    // [Snippet_getIntersection2D_line_1_0_0_-1_0_0]
    {
      EXPECT_TRUE(line.getIntersection2D(math::geometry::LineSegment(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(0).z(0),
        geometry_msgs::build<geometry_msgs::msg::Point>().x(-1).y(0).z(0))));
    }
    // [Snippet_getIntersection2D_line_1_0_0_-1_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_line_1_0_0_-1_0_0
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
