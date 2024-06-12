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

#ifndef GEOMETRY__QUATERNION__MAKE_QUATERNION_HPP_
#define GEOMETRY__QUATERNION__MAKE_QUATERNION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, typename U, typename V, typename W,
  std::enable_if_t<
    std::conjunction_v<std::is_scalar<T>, std::is_scalar<U>, std::is_scalar<V>, std::is_scalar<W>>,
    std::nullptr_t> = nullptr>
auto makeQuaternion(const T & x, const U & y, const V & z, const W & w)
{
  return geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(x).y(y).z(z).w(w);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__MAKE_QUATERNION_HPP_
