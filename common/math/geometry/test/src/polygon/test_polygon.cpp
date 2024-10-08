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
#include <scenario_simulator_exception/exception.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

TEST(Polygon, filterByAxis)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5.0, 2.0, 3.0), makePoint(1.0, 4.0, 5.0), makePoint(-1.0, 2.0, -3.0)};

  std::vector<double> values_x = math::geometry::filterByAxis(points, math::geometry::Axis::X);
  EXPECT_DOUBLE_EQ(values_x[0], 5.0);
  EXPECT_DOUBLE_EQ(values_x[1], 1.0);
  EXPECT_DOUBLE_EQ(values_x[2], -1.0);

  std::vector<double> values_y = math::geometry::filterByAxis(points, math::geometry::Axis::Y);
  EXPECT_DOUBLE_EQ(values_y[0], 2.0);
  EXPECT_DOUBLE_EQ(values_y[1], 4.0);
  EXPECT_DOUBLE_EQ(values_y[2], 2.0);

  std::vector<double> values_z = math::geometry::filterByAxis(points, math::geometry::Axis::Z);
  EXPECT_DOUBLE_EQ(values_z[0], 3.0);
  EXPECT_DOUBLE_EQ(values_z[1], 5.0);
  EXPECT_DOUBLE_EQ(values_z[2], -3.0);
}

TEST(Polygon, filterByAxisEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;

  std::vector<double> values_x = math::geometry::filterByAxis(points, math::geometry::Axis::X);
  EXPECT_EQ(values_x.size(), size_t(0));

  std::vector<double> values_y = math::geometry::filterByAxis(points, math::geometry::Axis::Y);
  EXPECT_EQ(values_y.size(), size_t(0));

  std::vector<double> values_z = math::geometry::filterByAxis(points, math::geometry::Axis::Z);
  EXPECT_EQ(values_z.size(), size_t(0));
}

TEST(Polygon, getMaxValue)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5.0, 2.0, 3.0), makePoint(1.0, 4.0, 5.0), makePoint(-1.0, 2.0, -3.0)};
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::X), 5.0);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Y), 4.0);
  EXPECT_DOUBLE_EQ(math::geometry::getMaxValue(points, math::geometry::Axis::Z), 5.0);
}

TEST(Polygon, getMaxValueEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::X), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::Y), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMaxValue(points, math::geometry::Axis::Z), common::SimulationError);
}

TEST(Polygon, getMinValue)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(5.0, 2.0, 3.0), makePoint(1.0, 4.0, 5.0), makePoint(-1.0, 2.0, -3.0)};
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::X), -1.0);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Y), 2.0);
  EXPECT_DOUBLE_EQ(math::geometry::getMinValue(points, math::geometry::Axis::Z), -3.0);
}

TEST(Polygon, getMinValueEmptyVector)
{
  std::vector<geometry_msgs::msg::Point> points;
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::X), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::Y), common::SimulationError);
  EXPECT_THROW(
    math::geometry::getMinValue(points, math::geometry::Axis::Z), common::SimulationError);
}

TEST(Polygon, get2DConvexHull)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(2.0, 2.0), makePoint(-2.0, 2.0), makePoint(-2.0, -2.0), makePoint(-1.0, 0.0)};
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), static_cast<size_t>(4));
  EXPECT_POINT_EQ(hull[0], points[2]);
  EXPECT_POINT_EQ(hull[1], points[1]);
  EXPECT_POINT_EQ(hull[2], points[0]);
  EXPECT_POINT_EQ(hull[3], points[2]);
}

TEST(Polygon, get2DConvexHullIdle)
{
  std::vector<geometry_msgs::msg::Point> points{
    makePoint(2.0, 2.0), makePoint(-2.0, 2.0), makePoint(-2.0, -2.0), makePoint(2.0, -2.0)};
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), static_cast<size_t>(5));
  EXPECT_POINT_EQ(hull[0], points[2]);
  EXPECT_POINT_EQ(hull[1], points[1]);
  EXPECT_POINT_EQ(hull[2], points[0]);
  EXPECT_POINT_EQ(hull[3], points[3]);
  EXPECT_POINT_EQ(hull[4], points[2]);
}

TEST(Polygon, get2DConvexHullEmpty)
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto hull = math::geometry::get2DConvexHull(points);
  EXPECT_EQ(hull.size(), size_t(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
