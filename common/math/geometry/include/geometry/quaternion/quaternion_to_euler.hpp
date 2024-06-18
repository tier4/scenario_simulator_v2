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

#ifndef GEOMETRY__QUATERNION__QUATERNION_TO_EULER_HPP_
#define GEOMETRY__QUATERNION__QUATERNION_TO_EULER_HPP_

#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>>, std::nullptr_t> = nullptr>
auto convertQuaternionToEulerAngle(const T & q)
{
  tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(tf_quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(roll).y(pitch).z(yaw);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__QUATERNION_TO_EULER_HPP_
