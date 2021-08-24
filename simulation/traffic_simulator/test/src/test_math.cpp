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
#include <traffic_simulator/math/bounding_box.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <traffic_simulator/math/distance.hpp>
#include <traffic_simulator/math/polynomial_solver.hpp>
#include <traffic_simulator/math/uuid.hpp>

TEST(Math, BoundingBox0)
{
  geometry_msgs::msg::Pose pose0;
  openscenario_msgs::msg::BoundingBox bbox0;
  bbox0.dimensions.x = 3;
  bbox0.dimensions.y = 3;
  bbox0.dimensions.z = 3;
  geometry_msgs::msg::Pose pose1;
  openscenario_msgs::msg::BoundingBox bbox1;
  bbox1.dimensions.x = 1;
  bbox1.dimensions.y = 1;
  bbox1.dimensions.z = 1;
  EXPECT_EQ(traffic_simulator::math::getPolygonDistance(pose0, bbox0, pose1, bbox1), boost::none);
}

TEST(Math, BoundingBox1)
{
  geometry_msgs::msg::Pose pose0;
  openscenario_msgs::msg::BoundingBox bbox0;
  bbox0.dimensions.x = 3;
  bbox0.dimensions.y = 3;
  bbox0.dimensions.z = 3;
  geometry_msgs::msg::Pose pose1;
  pose1.position.y = 5;
  openscenario_msgs::msg::BoundingBox bbox1;
  bbox1.dimensions.x = 1;
  bbox1.dimensions.y = 1;
  bbox1.dimensions.z = 1;
  EXPECT_TRUE(traffic_simulator::math::getPolygonDistance(pose0, bbox0, pose1, bbox1));
  EXPECT_DOUBLE_EQ(
    traffic_simulator::math::getPolygonDistance(pose0, bbox0, pose1, bbox1).get(), 3.0);
}

TEST(Math, UUID)
{
  EXPECT_STREQ(
    traffic_simulator::math::generateUUID("test").c_str(),
    traffic_simulator::math::generateUUID("test").c_str());
}

TEST(Math, Distance0)
{
  geometry_msgs::msg::Point p0, p1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 0);
  p1.x = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 1);
  p0.y = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), std::sqrt(2));
}

TEST(Math, Distance1)
{
  geometry_msgs::msg::Pose p0, p1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 0);
  p1.position.x = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 1);
  p0.position.y = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), std::sqrt(2));
}

TEST(Math, Distance2)
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Pose p1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 0);
  p1.position.x = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 1);
  p0.y = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), std::sqrt(2));
}

TEST(Math, Distance3)
{
  geometry_msgs::msg::Pose p0;
  geometry_msgs::msg::Point p1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 0);
  p1.x = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), 1);
  p0.position.y = 1;
  EXPECT_DOUBLE_EQ(traffic_simulator::math::getDistance(p0, p1), std::sqrt(2));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
