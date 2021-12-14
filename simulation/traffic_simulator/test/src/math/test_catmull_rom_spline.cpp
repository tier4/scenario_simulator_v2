// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>

TEST(CatmullRomSpline, GetCollisionPointIn2D)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  auto points = {p0, p1, p2};
  auto spline = traffic_simulator::math::CatmullRomSpline(points);
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
    EXPECT_DOUBLE_EQ(collision_s.get(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D({start, goal}, false);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.get(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D(start, goal, true);
  EXPECT_TRUE(collision_s);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.get(), 0.1);
  }
  collision_s = spline.getCollisionPointIn2D({start, goal}, true);
  if (collision_s) {
    EXPECT_DOUBLE_EQ(collision_s.get(), 0.1);
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
  EXPECT_NO_THROW(auto spline = traffic_simulator::math::CatmullRomSpline(points));
  auto spline = traffic_simulator::math::CatmullRomSpline(points);
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
  EXPECT_NO_THROW(auto spline = traffic_simulator::math::CatmullRomSpline(points));
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
  EXPECT_NO_THROW(auto spline = traffic_simulator::math::CatmullRomSpline(points));
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
  auto spline = traffic_simulator::math::CatmullRomSpline(points);
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
  auto spline = traffic_simulator::math::CatmullRomSpline(points);
  geometry_msgs::msg::Pose p;
  p.position.x = 0.1;
  p.position.y = 0;
  p.position.z = 0;
  const auto result = spline.getSValue(p);
  EXPECT_TRUE(result);
  EXPECT_TRUE(result.get() > 0.099);
  EXPECT_TRUE(result.get() < 0.101);
  p.position.x = 10;
  p.position.y = 0;
  p.position.z = 0;
  EXPECT_FALSE(spline.getSValue(p, 3));
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
  auto spline = traffic_simulator::math::CatmullRomSpline(points);
  auto trajectory = spline.getTrajectory(0, 3, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_DOUBLE_EQ(trajectory[0].x, 0);
  EXPECT_DOUBLE_EQ(trajectory[1].x, 1);
  EXPECT_DOUBLE_EQ(trajectory[2].x, 2);
  EXPECT_DOUBLE_EQ(trajectory[3].x, 3);
  trajectory = spline.getTrajectory(3, 0, 1.0);
  EXPECT_EQ(trajectory.size(), static_cast<size_t>(4));
  EXPECT_DOUBLE_EQ(trajectory[0].x, 3);
  EXPECT_DOUBLE_EQ(trajectory[1].x, 2);
  EXPECT_DOUBLE_EQ(trajectory[2].x, 1);
  EXPECT_DOUBLE_EQ(trajectory[3].x, 0);
}

TEST(CatmullRomSpline, CheckThrowingErrorWhenTheControlPointisAreNotEnough)
{
  EXPECT_THROW(
    traffic_simulator::math::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>(0)),
    common::SemanticError);
  EXPECT_THROW(
    traffic_simulator::math::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>(1)),
    common::SemanticError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
