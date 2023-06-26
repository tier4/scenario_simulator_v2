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
     * @note
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
    /// @note if s = 1, and autoscale = false, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(1, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note if s is not in range s = [0,1], getPoint function should be throw error.
    EXPECT_THROW(line.getPoint(2, false), common::SimulationError);
    EXPECT_THROW(line.getPoint(2, true), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, false), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, true), common::SimulationError);
    /// @note if s = std::hypot((0,0,0), (1,1,1)), and autoscale = true, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(std::sqrt(3), true),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
  }
}

TEST(LineSegment, GetPose)
{
  {
    /**
     * @note
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
     * @note
     * y
     * ^
     * |
     * + (x,y,z) = (0,1,0) => control point for spline, start point of line segment.
     * $
     * $
     * $
     * +----------------------> x
     * $
     * $
     * $
     * +
     * 
     * -----------------------------------------------------------
     * $$$$$$$$$$$$$ Line segment
     */
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0));
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0)));
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(2).z(0)));
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-2).z(0)));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
