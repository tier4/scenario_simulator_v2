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

#include <geometry/axis/axis.hpp>
#include <geometry/vector3/is_like_vector3.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
// Rotate a vector by a given angle around a specified axis
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeVector3<T>>, std::nullptr_t> = nullptr>
auto rotate(T & v, const double angle, const Axis axis)
{
  if (!std::isfinite(angle)) {
    THROW_SIMULATION_ERROR("The provided angle for rotation is not finite.");
  } else {
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(angle, axisToEigenAxis(axis)));
    const Eigen::Vector3d eigen_vector(v.x, v.y, v.z);
    const Eigen::Vector3d rotated_vector = rotation * eigen_vector;

    v.x = rotated_vector.x();
    v.y = rotated_vector.y();
    v.z = rotated_vector.z();
  }
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__ROTATE_HPP_
