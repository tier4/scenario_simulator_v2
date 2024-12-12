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

#ifndef GEOMETRY__VECTOR3__GET_NORMAL_VECTOR_HPP_
#define GEOMETRY__VECTOR3__GET_NORMAL_VECTOR_HPP_

#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>>, std::nullptr_t> = nullptr>
auto getNormalVector(const T & orientation) -> geometry_msgs::msg::Vector3
{
  const Eigen::Matrix3d rotation_matrix = getRotationMatrix(orientation);

  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(rotation_matrix(0, 2))
    .y(rotation_matrix(1, 2))
    .z(rotation_matrix(2, 2));
}

}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__GET_NORMAL_VECTOR_HPP_
