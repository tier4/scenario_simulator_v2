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

#ifndef GEOMETRY__QUATERNION__OPERATOR_HPP_
#define GEOMETRY__QUATERNION__OPERATOR_HPP_

#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/vector3/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
    nullptr>
auto operator+(const T & a, const U & b)
{
  auto v = T();
  v.x = a.x + b.x;
  v.y = a.y + b.y;
  v.z = a.z + b.z;
  v.w = a.w + b.w;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
    nullptr>
auto operator-(const T & a, const U & b)
{
  auto v = T();
  v.x = a.x - b.x;
  v.y = a.y - b.y;
  v.z = a.z - b.z;
  v.w = a.w - b.w;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
    nullptr>
auto operator*(const T & a, const U & b)
{
  auto v = T();
  v.x = a.w * b.x - a.z * b.y + a.y * b.z + a.x * b.w;
  v.y = a.z * b.x + a.w * b.y - a.x * b.z + a.y * b.w;
  v.z = -a.y * b.x + a.x * b.y + a.w * b.z + a.z * b.w;
  v.w = -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator*(const T & a, const U & b)
{
  T b_quat;
  b_quat.x = b.x;
  b_quat.y = b.y;
  b_quat.z = b.z;
  b_quat.w = 0.0;

  T a_inv = a;
  a_inv.x = -a.x;
  a_inv.y = -a.y;
  a_inv.z = -a.z;
  a_inv.w = a.w;

  T result_quat = a * b_quat * a_inv;

  U rotated_vector;
  rotated_vector.x = result_quat.x;
  rotated_vector.y = result_quat.y;
  rotated_vector.z = result_quat.z;

  return rotated_vector;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>>, std::nullptr_t> =
    nullptr>
auto operator+=(T & a, const U & b) -> decltype(auto)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  a.w += b.w;
  return a;
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__OPERATOR_HPP_
