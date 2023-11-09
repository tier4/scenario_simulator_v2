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
#include <geometry/vector3/operator.hpp>  // this header is neaded for truncate to work with geometry_msgs::msg::Vector3
#include <geometry/vector3/truncate.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

constexpr double EPS = 1e-6;

TEST(Vector3, truncate_msgVectorBelowMax)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(4.0, 4.0, 4.0);
  EXPECT_VECTOR3_EQ(math::geometry::truncate(vec0, 16), vec0)
}

TEST(Vector3, truncate_msgVectorOverMax)
{
  geometry_msgs::msg::Vector3 vec0 = makeVector(4.0, 4.0, 4.0);
  EXPECT_VECTOR3_EQ(math::geometry::truncate(vec0, std::sqrt(12.0)), makeVector(2.0, 2.0, 2.0));
}
