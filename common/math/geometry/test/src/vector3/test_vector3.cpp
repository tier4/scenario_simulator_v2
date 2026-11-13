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
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/internal_angle.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

constexpr double EPS = 1e-6;

/**
 * @brief Custom Vector3 struct using T type
 */
template <typename T>
struct CustomVector3
{
  T x, y, z;
  CustomVector3() = default;
  CustomVector3(T x, T y, T z) : x(x), y(y), z(z) {}
};

TEST(Vector3, hypot_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_DOUBLE_EQ(math::geometry::hypot(vec0, vec1), 6.0);
}

TEST(Vector3, hypot_customVector)
{
  CustomVector3<size_t> vec0(1.0, 2.0, 3.0);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_DOUBLE_EQ(math::geometry::hypot(vec0, vec1), 6.0);
}

/**
 * @note Test function correctness with parameter that is ros message vector.
 */
TEST(Vector3, norm_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);

  EXPECT_DOUBLE_EQ(math::geometry::norm(vec0), std::sqrt(14.0));
}

TEST(Vector3, norm_customVector)
{
  CustomVector3<size_t> vec0(1u, 2u, 3u);

  EXPECT_DOUBLE_EQ(math::geometry::norm(vec0), std::sqrt(14.0));
}

TEST(Vector3, normalize_msgVector)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);

  const double norm = std::sqrt(14.0);
  EXPECT_VECTOR3_EQ(
    math::geometry::normalize(vec0), makeVector(1.0 / norm, 2.0 / norm, 3.0 / norm));

  geometry_msgs::msg::Vector3 vec1;
  EXPECT_DOUBLE_EQ(math::geometry::norm(vec1), 0.0);

  geometry_msgs::msg::Vector3 vec2 = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::norm(vec2), std::sqrt(10.0));
}

TEST(Vector3, normalize_customVector)
{
  CustomVector3<float> vec0(1.0, 2.0, 3.0);

  const double norm = std::sqrt(14.0);
  EXPECT_VECTOR3_NEAR(
    math::geometry::normalize(vec0), makeVector(1.0 / norm, 2.0 / norm, 3.0 / norm), EPS);
}

TEST(Vector3, addition_msgVector)
{
  using math::geometry::operator+;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_VECTOR3_EQ((vec0 + vec1), makeVector(0.0, 0.0, 2.0));
}

TEST(Vector3, addition_customVector)
{
  using math::geometry::operator+;

  CustomVector3<size_t> vec0(1u, 2u, 3u);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_VECTOR3_EQ((vec0 + vec1), makeVector(0.0, 0.0, 2.0));
}

TEST(Vector3, subtraction_msgVector)
{
  using math::geometry::operator-;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_VECTOR3_EQ((vec0 - vec1), makeVector(2.0, 4.0, 4.0));
}

TEST(Vector3, subtraction_CustomVector)
{
  using math::geometry::operator-;

  CustomVector3<size_t> vec0(1u, 2u, 3u);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);

  EXPECT_VECTOR3_EQ((vec0 - vec1), makeVector(2.0, 4.0, 4.0));
}

TEST(Vector3, multiplication_msgVector)
{
  using math::geometry::operator*;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);

  EXPECT_VECTOR3_EQ((vec0 * 3.0), makeVector(3.0, 6.0, 9.0));
}

TEST(Vector3, multiplication_CustomVector)
{
  using math::geometry::operator*;

  CustomVector3<size_t> vec0(1u, 2u, 3u);

  EXPECT_VECTOR3_EQ((vec0 * 3.0), makeVector(3.0, 6.0, 9.0));
}

TEST(Vector3, division_msgVector)
{
  using math::geometry::operator/;

  geometry_msgs::msg::Vector3 vec0 = makeVector(2.0, 14.0, 6.0);

  EXPECT_VECTOR3_EQ((vec0 / 2.0), makeVector(1.0, 7.0, 3.0));
}

TEST(Vector3, division_CustomVector)
{
  using math::geometry::operator/;

  CustomVector3<size_t> vec0(2u, 14u, 6u);

  EXPECT_VECTOR3_EQ((vec0 / 2.0), makeVector(1.0, 7.0, 3.0));
}

TEST(Vector3, additionAssignment_msgVector)
{
  using math::geometry::operator+=;

  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 2.0, 3.0);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);
  vec0 += vec1;

  EXPECT_VECTOR3_EQ(vec0, makeVector(0.0, 0.0, 2.0));
}

TEST(Vector3, additionAssignment_customVector)
{
  using math::geometry::operator+=;

  CustomVector3<size_t> vec0(1u, 2u, 3u);
  geometry_msgs::msg::Vector3 vec1 = makeVector(-1.0, -2.0, -1.0);
  vec0 += vec1;

  EXPECT_VECTOR3_EQ(vec0, makeVector(0.0, 0.0, 2.0));
}

/**
 * @note Test function correctness when one of the vectors has length of 0.
 */
TEST(Vector3, normalize_zeroLength)
{
  geometry_msgs::msg::Vector3 vec;
  EXPECT_THROW(math::geometry::normalize(vec), common::SimulationError);
}

TEST(Vector3, normalize)
{
  geometry_msgs::msg::Vector3 vec = makeVector(1.0, 0.0, 3.0);
  vec = math::geometry::normalize(vec);

  geometry_msgs::msg::Vector3 ans = makeVector(0.31622776601683794, 0.0, 0.94868329805051377);
  EXPECT_VECTOR3_EQ(vec, ans);
  EXPECT_DOUBLE_EQ(math::geometry::norm(vec), 1.0);
}

TEST(Vector3, innerProduct_getInnerProduct)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0), vec1 = makeVector(-1.0, 0.0, -3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec1), -10.0);
}

TEST(Vector3, innerProduct_identical)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec0), 10.0);
}

TEST(Vector3, innerProduct_zero)
{
  geometry_msgs::msg::Vector3 vec0, vec1 = makeVector(1.0, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(math::geometry::innerProduct(vec0, vec1), 0.0);
}

TEST(Vector3, innerAngle_getInnerAngle)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0), vec1 = makeVector(-1.0, 0.0, -3.0);
  EXPECT_NEAR(math::geometry::getInternalAngle(vec0, vec1), M_PI, EPS);
}

TEST(Vector3, innerAngle_angleIdentical)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(1.0, 0.0, 3.0);
  EXPECT_NEAR(math::geometry::getInternalAngle(vec0, vec0), 0.0, EPS);
}

TEST(Vector3, innerAngle_angleZero)
{
  geometry_msgs::msg::Vector3 vec0, vec1 = makeVector(1.0, 0.0, 3.0);
  EXPECT_THROW(math::geometry::getInternalAngle(vec0, vec1), common::SimulationError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
