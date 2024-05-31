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

#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/quaternion/make_quaternion.hpp>
#include <geometry/quaternion/operator.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-6;

TEST(Quaternion, testCase1)
{
  using math::geometry::operator+;

  auto q1 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto q2 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto ans = q1 + q2;
  EXPECT_QUATERNION_EQ(ans, math::geometry::makeQuaternion(0, 2, 0, 2))
}

TEST(Quaternion, testCase2)
{
  using math::geometry::operator-;

  auto q1 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto q2 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto ans = q1 - q2;
  EXPECT_QUATERNION_EQ(ans, math::geometry::makeQuaternion(0, 0, 0, 0))
}

TEST(Quaternion, testCase3)
{
  using math::geometry::operator*;

  auto q1 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto q2 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto ans = q1 * q2;
  EXPECT_QUATERNION_EQ(ans, math::geometry::makeQuaternion(0, 2, 0, 0))
}

TEST(Quaternion, testCase4)
{
  using math::geometry::operator+=;

  auto q1 = math::geometry::makeQuaternion(0, 1, 0, 1);
  auto q2 = math::geometry::makeQuaternion(0, 1, 0, 1);
  q1 += q2;
  EXPECT_QUATERNION_EQ(q1, math::geometry::makeQuaternion(0, 2, 0, 2))
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
