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

#ifndef GEOMETRY__QUATERNION__NORM_HPP_
#define GEOMETRY__QUATERNION__NORM_HPP_

#include <cmath>
#include <geometry/quaternion/is_like_quaternion.hpp>

namespace math
{
namespace geometry
{
template <typename T, std::enable_if_t<IsLikeQuaternion<T>::value, std::nullptr_t> = nullptr>
auto norm(const T & q)
{
  return std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__NORM_HPP_
