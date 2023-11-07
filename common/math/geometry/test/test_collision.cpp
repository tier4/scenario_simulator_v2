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

#include <geometry/intersection/collision.hpp>
#include <scenario_simulator_exception/exception.hpp>

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Pose makePose(
  double x, double y, double z = 0.0,
  geometry_msgs::msg::Quaternion q = geometry_msgs::msg::Quaternion())
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = q;
  return pose;
}

traffic_simulator_msgs::msg::BoundingBox makeBbox(
  double dim_x, double dim_y, double dim_z = 0.0, double center_x = 0.0, double center_y = 0.0,
  double center_z = 0.0)
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = dim_x;
  bbox.dimensions.y = dim_y;
  bbox.dimensions.z = dim_z;
  bbox.center.x = center_x;
  bbox.center.y = center_y;
  bbox.center.z = center_z;
  return bbox;
}

TEST(Collision, DifferentHeight)
{
  geometry_msgs::msg::Pose pose0;
  geometry_msgs::msg::Pose pose1 = makePose(0, 0, 30.0);
  traffic_simulator_msgs::msg::BoundingBox box = makeBbox(1.0, 1.0, 1.0);
  EXPECT_FALSE(math::geometry::checkCollision2D(pose0, box, pose1, box));
}

TEST(Collision, SamePosition)
{
  geometry_msgs::msg::Pose pose0;
  geometry_msgs::msg::Pose pose1;
  traffic_simulator_msgs::msg::BoundingBox box = makeBbox(1.0, 1.0, 1.0);
  EXPECT_TRUE(math::geometry::checkCollision2D(pose0, box, pose1, box));
}

TEST(Collision, SameHeightNoCollision)
{
  geometry_msgs::msg::Pose pose0 = makePose(0, 0, 30.0);
  geometry_msgs::msg::Pose pose1;
  traffic_simulator_msgs::msg::BoundingBox box = makeBbox(1.0, 1.0, 1.0);
  EXPECT_FALSE(math::geometry::checkCollision2D(pose0, box, pose1, box));
}

TEST(Collision, Touching)
{
  geometry_msgs::msg::Pose pose0;
  geometry_msgs::msg::Pose pose1 = makePose(1.0, 1.0, 1.0);
  traffic_simulator_msgs::msg::BoundingBox box = makeBbox(1.0, 1.0, 1.0);
  EXPECT_TRUE(math::geometry::checkCollision2D(pose0, box, pose1, box));
}

TEST(Collision, PointInside)
{
  std::vector<geometry_msgs::msg::Point> polygon(4);
  polygon[1] = makePoint(1.0, 0.0);
  polygon[2] = makePoint(1.0, 1.0);
  polygon[3] = makePoint(0.0, 1.0);
  geometry_msgs::msg::Point point = makePoint(0.5, 0.5);
  EXPECT_TRUE(math::geometry::contains(polygon, point));
}

TEST(Collision, PointOutside)
{
  std::vector<geometry_msgs::msg::Point> polygon(4);
  polygon[1] = makePoint(1.0, 0.0);
  polygon[2] = makePoint(1.0, 1.0);
  polygon[3] = makePoint(0.0, 1.0);
  geometry_msgs::msg::Point point = makePoint(1.5, 0.5);
  EXPECT_FALSE(math::geometry::contains(polygon, point));
}

TEST(Collision, Line)
{
  std::vector<geometry_msgs::msg::Point> polygon(2);
  polygon[1] = makePoint(1.0, 1.0);
  geometry_msgs::msg::Point point = makePoint(0.5, 0.5);

  bool ans = true;
  EXPECT_NO_THROW(ans = math::geometry::contains(polygon, point));
  EXPECT_FALSE(ans);
}

TEST(Collision, EmptyVector)
{
  std::vector<geometry_msgs::msg::Point> polygon;
  geometry_msgs::msg::Point point = makePoint(0.5, 0.5);

  bool ans = true;
  EXPECT_NO_THROW(ans = math::geometry::contains(polygon, point));
  EXPECT_FALSE(ans);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
