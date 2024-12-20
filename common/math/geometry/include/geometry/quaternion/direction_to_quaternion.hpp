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

#ifndef GEOMETRY__QUATERNION__DIRECTION_TO_QUATERNION_HPP_
#define GEOMETRY__QUATERNION__DIRECTION_TO_QUATERNION_HPP_

#include <cmath>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/vector3/is_like_vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace math
{
namespace geometry
{
auto convertDirectionToQuaternion(const double dx, const double dy, const double dz)
{
  const auto euler_angles = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                              .x(0.0)
                              .y(std::atan2(-dz, std::hypot(dx, dy)))
                              .z(std::atan2(dy, dx));
  return math::geometry::convertEulerAngleToQuaternion(euler_angles);
}

template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeVector3<T>>, std::nullptr_t> = nullptr>
auto convertDirectionToQuaternion(const T & v)
{
  return convertDirectionToQuaternion(v.x, v.y, v.z);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__DIRECTION_TO_QUATERNION_HPP_
