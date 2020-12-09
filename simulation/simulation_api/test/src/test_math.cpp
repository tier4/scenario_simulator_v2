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

#include <simulation_api/math/catmull_rom_spline.hpp>
#include <simulation_api/math/hermite_curve.hpp>

TEST(Math, HermiteCurve1)
{
  geometry_msgs::msg::Pose start_pose, goal_pose;
  geometry_msgs::msg::Vector3 start_vec, goal_vec;
  goal_pose.position.x = 1;
  start_vec.x = 1;
  goal_vec.x = 1;
  simulation_api::math::HermiteCurve curve
    (start_pose, goal_pose, start_vec, goal_vec);
  EXPECT_DOUBLE_EQ(curve.getLength(), 1);
  EXPECT_DOUBLE_EQ(curve.getPoint(0.5, false).x, 0.5);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x, 1);
  EXPECT_DOUBLE_EQ(curve.getMaximu2DCurvature(), 0);
  geometry_msgs::msg::Point p;
  p.x = 0.1;
  p.y = 0;
  p.z = 0;
  EXPECT_TRUE(curve.getSValue(p, true));
  EXPECT_TRUE((curve.getSValue(p, true).get() > 0.099) &&
    (curve.getSValue(p, true).get() < 0.101));
}

TEST(Math, CatmullRomSpline1)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  auto points = {p0, p1, p2};
  auto spline = simulation_api::math::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 2);
}

TEST(Math, CatmullRomSpline2)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  p1.y = 3;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  p2.y = 5;
  auto points = {p0, p1, p2};
  EXPECT_NO_THROW(auto spline = simulation_api::math::CatmullRomSpline(points));
}

TEST(Math, CatmullRomSpline3)
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
  EXPECT_NO_THROW(auto spline = simulation_api::math::CatmullRomSpline(points));
}

TEST(Math, CatmullRomSpline4)
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
  EXPECT_NO_THROW(auto spline = simulation_api::math::CatmullRomSpline(points));
}

TEST(Math, CatmullRomSpline5)
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
  auto spline = simulation_api::math::CatmullRomSpline(points);
  EXPECT_DOUBLE_EQ(spline.getLength(), 4);
  auto point = spline.getPoint(3);
  EXPECT_DOUBLE_EQ(point.x, 3);
  EXPECT_DOUBLE_EQ(point.y, 0);
  EXPECT_DOUBLE_EQ(point.z, 0);
}

TEST(Math, CatmullRomSpline6)
{
  geometry_msgs::msg::Point p0;
  p0.x = -30.9281;
  p0.y = -23.1708;
  p0.z = -0.132544;
  geometry_msgs::msg::Point p1;
  p1.x = -29.2938;
  p1.y = -22.2938;
  p1.z = -0.162124;
  geometry_msgs::msg::Point p2;
  p2.x = -27.6596;
  p2.y = -21.4167;
  p2.z = -0.191704;
  geometry_msgs::msg::Point p3;
  p3.x = -33.0324;
  p3.y = -92.7566;
  p3.z = 2.28524;
  auto points = {p0, p1, p2, p3};
  auto spline = simulation_api::math::CatmullRomSpline(points);
}

TEST(Math, CatmullRomSpline7)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  geometry_msgs::msg::Point p3;
  p3.x = 4;
  auto points = {p0, p1, p2, p3};
  auto spline = simulation_api::math::CatmullRomSpline(points);
  geometry_msgs::msg::Point p;
  p.x = 0.1;
  p.y = 0;
  p.z = 0;
  EXPECT_TRUE(spline.getSValue(p));
  // std::cout << "result = " << spline.getSValue(p).get() << std::endl;
  EXPECT_TRUE(spline.getSValue(p).get() > 0.099);
  EXPECT_TRUE(spline.getSValue(p).get() < 0.101);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
