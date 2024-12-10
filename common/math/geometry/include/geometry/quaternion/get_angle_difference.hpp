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

#ifndef GEOMETRY__QUATERNION__GET_ANGLE_DIFFERENCE_HPP_
#define GEOMETRY__QUATERNION__GET_ANGLE_DIFFERENCE_HPP_

#include <Eigen/Geometry>
#include <geometry/quaternion/is_like_quaternion.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>>, std::nullptr_t> = nullptr>
auto getAngleDifference(const T & quat1, const T & quat2) -> double
{
  const Eigen::Quaterniond q1(quat1.w, quat1.x, quat1.y, quat1.z);
  const Eigen::Quaterniond q2(quat2.w, quat2.x, quat2.y, quat2.z);

  const Eigen::AngleAxisd delta(q1.inverse() * q2);

  return std::abs(delta.angle());  // [rad]
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__GET_ANGLE_DIFFERENCE_HPP_
