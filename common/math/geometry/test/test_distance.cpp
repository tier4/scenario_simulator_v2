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

#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <scenario_simulator_exception/exception.hpp>

TEST(Distance, PointToPoint)
{
  geometry_msgs::msg::Point p0, p1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 0);
  p1.x = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 1);
  p0.y = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), std::sqrt(2));
}

TEST(Distance, PoseToPose)
{
  geometry_msgs::msg::Pose p0, p1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 0);
  p1.position.x = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 1);
  p0.position.y = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), std::sqrt(2));
}

TEST(Distance, PointToPose)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Pose p1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 0);
  p1.position.x = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 1);
  p0.y = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), std::sqrt(2));
}

TEST(Distance, PoseToPoint)
{
  geometry_msgs::msg::Pose p0;
  geometry_msgs::msg::Point p1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 0);
  p1.x = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), 1);
  p0.position.y = 1;
  EXPECT_DOUBLE_EQ(math::geometry::getDistance(p0, p1), std::sqrt(2));
}

TEST(Distance, Distance2DDisjoint)
{
  std::vector<geometry_msgs::msg::Point> polygon0, polygon1;
  auto makePoint = [](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    return p;
  };
  polygon0.emplace_back(makePoint(0, 0));
  polygon0.emplace_back(makePoint(0, 1));
  polygon0.emplace_back(makePoint(1, 1));
  polygon0.emplace_back(makePoint(1, 0));
  polygon1.emplace_back(makePoint(2, 2));
  polygon1.emplace_back(makePoint(2, 3));
  polygon1.emplace_back(makePoint(3, 3));
  polygon1.emplace_back(makePoint(3, 2));
  EXPECT_DOUBLE_EQ(math::geometry::getDistance2D(polygon0, polygon1), std::sqrt(2));
}

TEST(Distance, Distance2DIntersect)
{
  std::vector<geometry_msgs::msg::Point> polygon0, polygon1;
  auto makePoint = [](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    return p;
  };
  polygon0.emplace_back(makePoint(0, 0));
  polygon0.emplace_back(makePoint(0, 2));
  polygon0.emplace_back(makePoint(2, 2));
  polygon0.emplace_back(makePoint(2, 0));
  polygon1.emplace_back(makePoint(1, 1));
  polygon1.emplace_back(makePoint(1, 3));
  polygon1.emplace_back(makePoint(3, 3));
  polygon1.emplace_back(makePoint(3, 1));
  EXPECT_DOUBLE_EQ(math::geometry::getDistance2D(polygon0, polygon1), 0);
}

TEST(Distance, Distance2DZeroElements)
{
  std::vector<geometry_msgs::msg::Point> polygon0, polygon1;
  auto makePoint = [](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    return p;
  };
  polygon0.emplace_back(makePoint(0, 0));
  polygon0.emplace_back(makePoint(0, 1));
  polygon0.emplace_back(makePoint(1, 1));
  polygon0.emplace_back(makePoint(1, 0));
  EXPECT_THROW(
    math::geometry::getDistance2D(polygon0, polygon1), boost::geometry::empty_input_exception);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
