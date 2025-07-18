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

#ifndef GEOMETRY__VECTOR3__OPERATOR_HPP_
#define GEOMETRY__VECTOR3__OPERATOR_HPP_

#include <Eigen/Dense>
#include <geometry/vector3/is_like_vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeVector3<T>>, std::nullptr_t> = nullptr>
auto operator+(const T & v, const Eigen::Vector3d & eigen_v) -> T
{
  T result;
  result.x = v.x + eigen_v.x();
  result.y = v.y + eigen_v.y();
  result.z = v.z + eigen_v.z();
  return result;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator+(const T & a, const U & b)
{
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
    geometry_msgs::msg::Vector3 v;
    v.x = a.x + b.x;
    v.y = a.y + b.y;
    v.z = a.z + b.z;
    return v;
  } else {
    geometry_msgs::msg::Point v;
    v.x = a.x + b.x;
    v.y = a.y + b.y;
    v.z = a.z + b.z;
    return v;
  }
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator-(const T & a, const U & b)
{
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
    geometry_msgs::msg::Vector3 v;
    v.x = a.x - b.x;
    v.y = a.y - b.y;
    v.z = a.z - b.z;
    return v;
  } else {
    geometry_msgs::msg::Point v;
    v.x = a.x - b.x;
    v.y = a.y - b.y;
    v.z = a.z - b.z;
    return v;
  }
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
    nullptr>
auto operator*(const T & a, const U & b)
{
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
    geometry_msgs::msg::Vector3 v;
    v.x = a.x * b;
    v.y = a.y * b;
    v.z = a.z * b;
    return v;
  } else {
    geometry_msgs::msg::Point v;
    v.x = a.x * b;
    v.y = a.y * b;
    v.z = a.z * b;
    return v;
  }
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator*(const T & a, const U & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, std::is_scalar<U>>, std::nullptr_t> =
    nullptr>
auto operator/(const T & a, const U & b)
{
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
    geometry_msgs::msg::Vector3 v;
    v.x = a.x / b;
    v.y = a.y / b;
    v.z = a.z / b;
    return v;
  } else {
    geometry_msgs::msg::Point v;
    v.x = a.x / b;
    v.y = a.y / b;
    v.z = a.z / b;
    return v;
  }
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator+=(T & a, const U & b) -> decltype(auto)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto operator==(const T & a, const U & b) -> bool
{
  constexpr decltype(a.x) e = std::numeric_limits<decltype(a.x)>::epsilon();
  return (std::abs(a.x - b.x) < e) && (std::abs(a.y - b.y) < e) && (std::abs(a.z - b.z) < e);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__OPERATOR_HPP_
