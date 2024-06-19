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

#ifndef GEOMETRY__QUATERNION__EULER_TO_QUATERNION_HPP_
#define GEOMETRY__QUATERNION__EULER_TO_QUATERNION_HPP_

#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <geometry/vector3/is_like_vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeVector3<T>>, std::nullptr_t> = nullptr>
auto convertEulerAngleToQuaternion(const T & v)
{
  const double & roll = v.x;
  const double & pitch = v.y;
  const double & yaw = v.z;
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(roll, pitch, yaw);
  return geometry_msgs::build<geometry_msgs::msg::Quaternion>()
    .x(tf_quat.x())
    .y(tf_quat.y())
    .z(tf_quat.z())
    .w(tf_quat.w());
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__EULER_TO_QUATERNION_HPP_
