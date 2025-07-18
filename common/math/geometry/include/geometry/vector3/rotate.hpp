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

#ifndef GEOMETRY__VECTOR3__ROTATE_HPP_
#define GEOMETRY__VECTOR3__ROTATE_HPP_

#include <Eigen/Geometry>
#include <geometry/quaternion/is_like_quaternion.hpp>
#include <geometry/vector3/is_like_vector3.hpp>

namespace math
{
namespace geometry
{
template <
  typename V, typename Q,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<V>, IsLikeQuaternion<Q>>, std::nullptr_t> =
    nullptr>
auto rotate(const V & vector, const Q & quaternion)
{
  const Eigen::Quaterniond eigen_quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  const Eigen::Vector3d eigen_vector(vector.x, vector.y, vector.z);
  const Eigen::Vector3d eigen_rotated_vector = eigen_quaternion * eigen_vector;
  V rotated_vector;
  rotated_vector.x = eigen_rotated_vector.x();
  rotated_vector.y = eigen_rotated_vector.y();
  rotated_vector.z = eigen_rotated_vector.z();
  return rotated_vector;
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__ROTATE_HPP_
