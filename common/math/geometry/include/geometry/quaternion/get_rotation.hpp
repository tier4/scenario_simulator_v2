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

#ifndef GEOMETRY__QUATERNION__GET_ROTATION_HPP_
#define GEOMETRY__QUATERNION__GET_ROTATION_HPP_

#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/quaternion/operator.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
    nullptr>
auto getRotation(T from, U to)
{
  // This function gets the conjugate of the quaternion.
  from.x *= -1;
  from.y *= -1;
  from.z *= -1;
  return from * to;
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__GET_ROTATION_HPP_
