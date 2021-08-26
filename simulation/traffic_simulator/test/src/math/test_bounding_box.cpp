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

TEST(BoundigBox, GetPolygonDistanceWithCollision)
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

TEST(BoundigBox, GetPolygonDistanceWithoutCollision)
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
