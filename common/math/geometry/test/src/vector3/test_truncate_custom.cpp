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
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/truncate.hpp>

#include "../expect_eq_macros.hpp"

constexpr double EPS = 1e-6;

/**
 * @brief Custom Vector3 struct using T type with multiplication and division operators
 */
template <typename T>
struct CustomVector3
{
  T x, y, z;
  CustomVector3() = default;
  CustomVector3(T x, T y, T z) : x(x), y(y), z(z) {}
  template <typename U>
  CustomVector3 operator*(U v) const
  {
    return CustomVector3(x * v, y * v, z * v);
  }
  template <typename U>
  CustomVector3 operator/(U v) const
  {
    return CustomVector3(x / v, y / v, z / v);
  }
};

TEST(Vector3, truncate_customVectorBelowMax)
{
  CustomVector3<float> vec0(4.0, 4.0, 4.0);
  EXPECT_VECTOR3_EQ(math::geometry::truncate(vec0, 16), vec0)
}

TEST(Vector3, truncate_customVectorOverMax)
{
  CustomVector3<float> vec0(4.0, 4.0, 4.0);
  EXPECT_VECTOR3_EQ(math::geometry::truncate(vec0, std::sqrt(12.0)), CustomVector3(2.0, 2.0, 2.0));
}
