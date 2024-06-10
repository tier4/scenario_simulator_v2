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

#ifndef GEOMETRY__VECTOR3__VECTOR3_HPP_
#define GEOMETRY__VECTOR3__VECTOR3_HPP_

#include <geometry/vector3/is_like_vector3.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, typename U, typename V,
  std::enable_if_t<
    std::conjunction_v<std::is_scalar<T>, std::is_scalar<U>, std::is_scalar<V>>, std::nullptr_t> =
    nullptr>
auto vector3(const T & x, const U & y, const V & z)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__VECTOR3_HPP_
