// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CONTEXT_GAMMA_PLANNER__UTILS_MATH_UTILS_HPP_
#define CONTEXT_GAMMA_PLANNER__UTILS_MATH_UTILS_HPP_

#include <cmath>
#include <geometry/vector3/operator.hpp>

using math::geometry::operator-;
using math::geometry::operator+;
using math::geometry::operator*;
using math::geometry::operator/;
using math::geometry::operator+=;

namespace context_gamma_planner
{
template <typename T>
auto sqr(const T a)
{
  return a * a;
}
template <typename T, typename U>
auto det(const T & a, const U & b)
{
  return a.x * b.y - a.y * b.x;
}
template <typename T>
auto normalize(const T & a)
{
  const auto n = std::hypot(a.x, a.y, a.z);
  if (n < 1e-6) {
    if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
      return geometry_msgs::msg::Vector3();
    } else {
      return geometry_msgs::msg::Point();
    }
  }
  return a / n;
}  // namespace math
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_MATH_UTILS_HPP_