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

#include "expect_eq_macros.hpp"

TEST(LineSegmentTest, GetPoint)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note if s = 0, the value should be start point.
    EXPECT_POINT_EQ(
      line.getPoint(0, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0));
    /// @note if s = 1, and autoscale = false, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(1, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note if s = std::sart(3), and autoscale = true, the value should be end point.
    EXPECT_POINT_EQ(
      line.getPoint(std::sqrt(3), true),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
