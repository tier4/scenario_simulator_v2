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

#ifndef GEOMETRY__QUATERNION__NORMALIZE_HPP_
#define GEOMETRY__QUATERNION__NORMALIZE_HPP_

#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/quaternion/norm.hpp>
#include <geometry/quaternion/operator.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
template <typename T, std::enable_if_t<IsLikeQuaternion<T>::value, std::nullptr_t> = nullptr>
auto normalize(const T & q)
{
  if (const auto n = norm(q); std::fabs(n) <= std::numeric_limits<double>::epsilon()) {
    THROW_SIMULATION_ERROR(
      "Norm of Quaternion(", q.w, ",", q.x, ",", q.y, ",", q.z, ") is ", n,
      ". The norm of the quaternion you want to normalize should be greater than ",
      std::numeric_limits<double>::epsilon());
  } else {
    return geometry_msgs::build<geometry_msgs::msg::Quaternion>()
      .x(q.x / n)
      .y(q.y / n)
      .z(q.z / n)
      .w(q.w / n);
  }
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__NORMALIZE_HPP_
