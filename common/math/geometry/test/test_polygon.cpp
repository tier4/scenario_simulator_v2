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

#include <geometry/polygon/polygon.hpp>

#include "expect_eq_macros.hpp"

TEST(Polygon, filterByAxis)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  {
    p0.x = 5.0;
    p0.y = 2.0;
    p0.z = 3.0;
  }
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  {
    p1.x = 1.0;
    p1.y = 4.0;
    p1.z = 5.0;
  }
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  {
    p2.x = -1.0;
    p2.y = 2.0;
    p2.z = -3.0;
  }
  points.emplace_back(p2);
  std::vector<double> values;
  values = math::geometry::filterByAxis(points, math::geometry::Axis::X);
  EXPECT_DOUBLE_EQ(values[0], 5);
  EXPECT_DOUBLE_EQ(values[1], 1);
  EXPECT_DOUBLE_EQ(values[2], -1);
  values = math::geometry::filterByAxis(points, math::geometry::Axis::Y);
  EXPECT_DOUBLE_EQ(values[0], 2);
  EXPECT_DOUBLE_EQ(values[1], 4);
  EXPECT_DOUBLE_EQ(values[2], 2);
  values = math::geometry::filterByAxis(points, math::geometry::Axis::Z);
  EXPECT_DOUBLE_EQ(values[0], 3);
  EXPECT_DOUBLE_EQ(values[1], 5);
  EXPECT_DOUBLE_EQ(values[2], -3);
}

TEST(Polygon, GetMinMaxValue)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  {
    p0.x = 5.0;
    p0.y = 2.0;
    p0.z = 3.0;
  }
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  {
    p1.x = 1.0;
    p1.y = 4.0;
    p1.z = 5.0;
  }
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  {
    p2.x = -1.0;
    p2.y = 2.0;
    p2.z = -3.0;
  }
  points.emplace_back(p2);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::X), 5);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::X), -1);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Y), 4);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Y), 2);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Z), 5);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Z), -3);
}

TEST(Polygon, get2DConvexHull)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  {
    p0.x = 2.0;
    p0.y = 2.0;
    p0.z = 0.0;
  }
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  {
    p1.x = -2.0;
    p1.y = 2.0;
    p1.z = 0.0;
  }
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  {
    p2.x = -2.0;
    p2.y = -2.0;
    p2.z = 0.0;
  }
  points.emplace_back(p2);
  geometry_msgs::msg::Point p3;
  {
    p3.x = -1.0;
    p3.y = 0.0;
    p3.z = 0.0;
  }
  points.emplace_back(p3);
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), static_cast<size_t>(4));
  EXPECT_POINT_EQ(hull[0], p2);
  EXPECT_POINT_EQ(hull[1], p1);
  EXPECT_POINT_EQ(hull[2], p0);
  EXPECT_POINT_EQ(hull[3], p2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
