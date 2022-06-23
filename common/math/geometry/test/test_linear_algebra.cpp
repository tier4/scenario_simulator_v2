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

#include <geometry/linear_algebra.hpp>

#include "expect_eq_macros.hpp"

TEST(LINEAR_ALGEBRA, GET_SIZE)
{
  geometry_msgs::msg::Vector3 vec;
  EXPECT_DOUBLE_EQ(math::geometrygetSize(vec), 0.0);
  vec.x = 1.0;
  vec.y = 0.0;
  vec.z = 3.0;
  EXPECT_DOUBLE_EQ(math::geometrygetSize(vec), std::sqrt(10.0));
}

TEST(LINEAR_ALGEBRA, NORMALIZE)
{
  geometry_msgs::msg::Vector3 vec;
  EXPECT_THROW(math::geometrynormalize(vec), common::SimulationError);
  vec.x = 1.0;
  vec.y = 0.0;
  vec.z = 3.0;
  vec = math::geometrynormalize(vec);
  EXPECT_DOUBLE_EQ(vec.x, 0.31622776601683794);
  EXPECT_DOUBLE_EQ(vec.y, 0.0);
  EXPECT_DOUBLE_EQ(vec.z, 0.94868329805051377);
  EXPECT_DOUBLE_EQ(math::geometrygetSize(vec), 1.0);
}

TEST(LINEAR_ALGEBRA, MULTIPLY)
{
  geometry_msgs::msg::Vector3 vec = math::geometryvector3(0, 3, 1);
  vec * 1.0;
  EXPECT_VECTOR3_EQ((vec * 1.0), math::geometryvector3(0, 3, 1));
  EXPECT_VECTOR3_EQ((vec * 2.0), math::geometryvector3(0, 6, 2));
  EXPECT_VECTOR3_EQ((vec * 2.0), (2.0 * vec));
}

TEST(LINEAR_ALGEBRA, ADDITION)
{
  geometry_msgs::msg::Vector3 vec = math::geometryvector3(0, 3, 1);
  EXPECT_VECTOR3_EQ((vec + vec), (2.0 * vec));
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 3;
  p.z = 1;
  EXPECT_VECTOR3_EQ((p + vec), (2.0 * vec));
}

TEST(LINEAR_ALGEBRA, SUBTRACTION)
{
  geometry_msgs::msg::Vector3 vec = math::geometryvector3(0, 3, 1);
  EXPECT_VECTOR3_EQ((vec - vec), geometry_msgs::msg::Vector3());
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 3;
  p.z = 1;
  EXPECT_VECTOR3_EQ((p - vec), geometry_msgs::msg::Vector3());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
