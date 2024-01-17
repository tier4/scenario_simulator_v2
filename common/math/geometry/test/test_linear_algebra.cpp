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
#include <limits>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-3;

TEST(LINEAR_ALGEBRA, GET_SIZE_ZERO)
{
  geometry_msgs::msg::Vector3 vec;
  EXPECT_DOUBLE_EQ(math::geometry::getSize(vec), 0.0);
}

TEST(LINEAR_ALGEBRA, GET_SIZE)
{
  geometry_msgs::msg::Vector3 vec = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::getSize(vec), std::sqrt(10.0));
}

TEST(LINEAR_ALGEBRA, NORMALIZE_ZERO)
{
  geometry_msgs::msg::Vector3 vec;
  EXPECT_THROW(math::geometry::normalize(vec), common::SimulationError);
}

TEST(LINEAR_ALGEBRA, NORMALIZE)
{
  geometry_msgs::msg::Vector3 vec = makeVector(1.0, 0.0, 3.0);
  vec = math::geometry::normalize(vec);

  geometry_msgs::msg::Vector3 ans = makeVector(0.31622776601683794, 0.0, 0.94868329805051377);
  EXPECT_VECTOR3_EQ(vec, ans);
  EXPECT_DOUBLE_EQ(math::geometry::getSize(vec), 1.0);
}

TEST(LINEAR_ALGEBRA, INNER_PRODUCT)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0), vec1 = makeVector(-1.0, 0.0, -3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec1), -10.0);
}

TEST(LINEAR_ALGEBRA, INNER_PRODUCT_IDENTICAL)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec0), 10.0);
}

TEST(LINEAR_ALGEBRA, INNER_PRODUCT_ZERO)
{
  geometry_msgs::msg::Vector3 vec0, vec1 = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec1), 0.0);
}

TEST(LINEAR_ALGEBRA, GET_INTERNAL_ANGLE)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0), vec1 = makeVector(-1.0, 0.0, -3.0);
  EXPECT_NEAR(math::geometry::getInternalAngle(vec0, vec1), M_PI, EPS);
}

TEST(LINEAR_ALGEBRA, GET_INTERNAL_ANGLE_IDENTICAL)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0);
  EXPECT_NEAR(math::geometry::getInternalAngle(vec0, vec0), 0.0, EPS);
}

TEST(LINEAR_ALGEBRA, GET_INTERNAL_ANGLE_ZERO)
{
  geometry_msgs::msg::Vector3 vec0, vec1 = makeVector(1.0, 0.0, 3.0);
  EXPECT_THROW(math::geometry::getInternalAngle(vec0, vec1), common::SimulationError);
}

TEST(LINEAR_ALGEBRA, DIVIDE)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((vec / 2.0), makeVector(0.0, 1.5, 0.5));
}

TEST(LINEAR_ALGEBRA, DIVIDE_ZERO)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  vec = vec / 0.0;
  EXPECT_TRUE(std::isnan(vec.x));
  EXPECT_TRUE(std::isinf(vec.y));
  EXPECT_TRUE(std::isinf(vec.z));
}

TEST(LINEAR_ALGEBRA, MULTIPLY_FIRST)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((vec * 2.0), makeVector(0.0, 6.0, 2.0));
}

TEST(LINEAR_ALGEBRA, MULTIPLY_SECOND)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((2.0 * vec), makeVector(0.0, 6.0, 2.0));
}

TEST(LINEAR_ALGEBRA, ADDITION_POINT_VECTOR)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  geometry_msgs::msg::Point p = makePoint(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((p + vec), makeVector(0.0, 6.0, 2.0));
}

TEST(LINEAR_ALGEBRA, ADDITION_VECTOR_VECTOR)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((vec + vec), makeVector(0.0, 6.0, 2.0));
}

TEST(LINEAR_ALGEBRA, ADDITION_POINT_POINT)
{
  geometry_msgs::msg::Point p0 = makePoint(0.0, 3.0, 1.0), p1 = makePoint(2.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((p0 + p1), makePoint(2.0, 6.0, 2.0));
}

TEST(LINEAR_ALGEBRA, SUBTRACTION_POINT_VECTOR)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  geometry_msgs::msg::Point p = makePoint(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((p - vec), geometry_msgs::msg::Vector3());
}

TEST(LINEAR_ALGEBRA, SUBTRACTION_VECTOR_VECTOR)
{
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((vec - vec), geometry_msgs::msg::Vector3());
}

TEST(LINEAR_ALGEBRA, SUBTRACTION_POINT_POINT)
{
  geometry_msgs::msg::Point p0 = makePoint(0.0, 3.0, 1.0), p1 = makePoint(2.0, 3.0, 1.0);
  EXPECT_VECTOR3_EQ((p0 - p1), makePoint(-2.0, 0.0, 0.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
