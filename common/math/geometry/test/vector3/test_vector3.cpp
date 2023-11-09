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

#include <cmath>
#include <geometry/linear_algebra.hpp>
#include <geometry/test/expect_eq_macros.hpp>
#include <geometry/test/test_utils.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>

constexpr double EPS = 1e-6;

/**
 * @brief Custom Vector3 struct using T type
 */
template <typename T>
struct CustomVector3
{
  T x = 0, y = 0, z = 0;
  CustomVector3() = default;
  CustomVector3(T x, T y, T z) : x(x), y(y), z(z) {}
};

TEST(Vector3, hypot_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_DOUBLE_EQ(math::geometry::hypot(vec0, vec1), 6.0);
}

TEST(Vector3, hypot_customVector)
{
  CustomVector3<size_t> vec0(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_DOUBLE_EQ(math::geometry::hypot(vec0, vec1), 6.0);
}

TEST(Vector3, norm_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);

  EXPECT_DOUBLE_EQ(math::geometry::norm(vec0), std::sqrt(14.0));
}

TEST(Vector3, norm_customVector)
{
  CustomVector3<size_t> vec0(1, 2, 3);

  EXPECT_DOUBLE_EQ(math::geometry::norm(vec0), std::sqrt(14.0));
}

TEST(Vector3, normalize_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);

  const double norm = std::sqrt(14.0);
  EXPECT_VECTOR3_EQ(
    math::geometry::normalize(vec0), makeVector(1.0 / norm, 2.0 / norm, 3.0 / norm));
}

TEST(Vector3, normalize_customVector)
{
  CustomVector3<float> vec0(1, 2, 3);

  const double norm = std::sqrt(14.0);
  EXPECT_VECTOR3_NEAR(
    math::geometry::normalize(vec0), makeVector(1.0 / norm, 2.0 / norm, 3.0 / norm), EPS);
}

TEST(Vector3, addition_msgVector)
{
  using math::geometry::operator+;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_VECTOR3_EQ((vec0 + vec1), makeVector(0, 0, 2));
}

TEST(Vector3, addition_customVector)
{
  using math::geometry::operator+;

  CustomVector3<size_t> vec0(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_VECTOR3_EQ((vec0 + vec1), makeVector(0, 0, 2));
}

TEST(Vector3, subtraction_msgVector)
{
  using math::geometry::operator-;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_VECTOR3_EQ((vec0 - vec1), makeVector(2, 4, 4));
}

TEST(Vector3, subtraction_CustomVector)
{
  using math::geometry::operator-;

  CustomVector3<size_t> vec0(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);

  EXPECT_VECTOR3_EQ((vec0 - vec1), makeVector(2, 4, 4));
}

TEST(Vector3, multiplication_msgVector)
{
  using math::geometry::operator*;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);

  EXPECT_VECTOR3_EQ((vec0 * 3), makeVector(3, 6, 9));
}

TEST(Vector3, multiplication_CustomVector)
{
  using math::geometry::operator*;

  CustomVector3<size_t> vec0(1, 2, 3);

  EXPECT_VECTOR3_EQ((vec0 * 3), makeVector(3, 6, 9));
}

TEST(Vector3, division_msgVector)
{
  using math::geometry::operator/;

  geometry_msgs::msg::Vector3 vec0 = makeVector(2, 14, 6);

  EXPECT_VECTOR3_EQ((vec0 / 2), makeVector(1, 7, 3));
}

TEST(Vector3, division_CustomVector)
{
  using math::geometry::operator/;

  CustomVector3<size_t> vec0(2, 14, 6);

  EXPECT_VECTOR3_EQ((vec0 / 2), makeVector(1, 7, 3));
}

TEST(Vector3, additionAssignment_msgVector)
{
  using math::geometry::operator+=;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);
  vec0 += vec1;

  EXPECT_VECTOR3_EQ(vec0, makeVector(0, 0, 2));
}

TEST(Vector3, additionAssignment_customVector)
{
  using math::geometry::operator+=;

  CustomVector3<size_t> vec0(1, 2, 3);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1, -2, -1);
  vec0 += vec1;

  EXPECT_VECTOR3_EQ(vec0, makeVector(0, 0, 2));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
