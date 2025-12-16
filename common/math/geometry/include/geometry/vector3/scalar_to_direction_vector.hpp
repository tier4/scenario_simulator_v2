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

#ifndef GEOMETRY__VECTOR3__SCALAR_TO_DIRECTION_VECTOR_HPP_
#define GEOMETRY__VECTOR3__SCALAR_TO_DIRECTION_VECTOR_HPP_

#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/vector3.hpp>
#include <type_traits>

namespace math
{
namespace geometry
{
/// @brief converts scalar magnitude to 3D directional vector
/// @tparam T quaternion-like type (must have x, y, z, w members)
/// @param magnitude scalar magnitude (e.g., speed, force, distance)
/// @param orientation quaternion representing the direction (pitch and yaw are extracted)
/// @return 3D vector with given magnitude in the direction defined by orientation
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>>, std::nullptr_t> = nullptr>
auto scalarToDirectionVector(const double magnitude, const T & orientation)
{
  const auto pitch = -convertQuaternionToEulerAngle(orientation).y;
  const auto yaw = convertQuaternionToEulerAngle(orientation).z;
  return vector3(
    std::cos(pitch) * std::cos(yaw) * magnitude, std::cos(pitch) * std::sin(yaw) * magnitude,
    std::sin(pitch) * magnitude);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__SCALAR_TO_DIRECTION_VECTOR_HPP_
