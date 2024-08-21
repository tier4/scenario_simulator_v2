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

#ifndef GEOMETRY__QUATERNION__GET_ROTATION_MATRIX_HPP_
#define GEOMETRY__QUATERNION__GET_ROTATION_MATRIX_HPP_

#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <geometry/quaternion/is_like_quaternion.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace math
{
namespace geometry
{
template <
  typename T, std::enable_if_t<std::conjunction_v<IsLikeQuaternion<T>>, std::nullptr_t> = nullptr>
auto getRotationMatrix(T quat)
{
  auto x = quat.x;
  auto y = quat.y;
  auto z = quat.z;
  auto w = quat.w;
  Eigen::Matrix3d ret(3, 3);
  // clang-format off
  ret << x * x - y * y - z * z + w * w,  2 * (x * y - z * w),            2 * (z * x + w * y),
         2 * (x * y + z * w),           -x * x + y * y - z * z + w * w,  2 * (y * z - x * w), 
         2 * (z * x - w * y),            2 * (y * z + w * x),           -x * x - y * y + z * z + w * w;
  // clang-format on
  return ret;
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__GET_ROTATION_MATRIX_HPP_
