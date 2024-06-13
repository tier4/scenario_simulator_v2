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

#ifndef GEOMETRY__QUATERNION__SLERP_HPP_
#define GEOMETRY__QUATERNION__SLERP_HPP_

#include <cmath>
#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
namespace math
{
namespace geometry
{
template <
  typename T, typename U, typename V,
  std::enable_if_t<
    std::conjunction_v<IsLikeQuaternion<T>, IsLikeQuaternion<U>, std::is_scalar<V>>,
    std::nullptr_t> = nullptr>
auto slerp(T quat1, U quat2, V t)
{
  double qr = quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
  double ss = 1.0 - qr * qr;
  constexpr double e = std::numeric_limits<double>::epsilon();
  if (std::fabs(ss) <= e) {
    return geometry_msgs::build<geometry_msgs::msg::Quaternion>()
      .x(quat1.x)
      .y(quat1.y)
      .z(quat1.z)
      .w(quat1.w);
  } else {
    double sp = std::sqrt(ss);
    double ph = std::acos(qr);
    double pt = ph * t;
    double t1 = std::sin(pt) / sp;
    double t0 = std::sin(ph - pt) / sp;

    return geometry_msgs::build<geometry_msgs::msg::Quaternion>()
      .x(quat1.x * t0 + quat2.x * t1)
      .y(quat1.y * t0 + quat2.y * t1)
      .z(quat1.z * t0 + quat2.z * t1)
      .w(quat1.w * t0 + quat2.w * t1);
  }
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__SLERP_HPP_
