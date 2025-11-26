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

#ifndef GEOMETRY__VECTOR3__IS_FINITE_HPP_
#define GEOMETRY__VECTOR3__IS_FINITE_HPP_

#include <geometry/vector3/is_like_vector3.hpp>

namespace math
{
namespace geometry
{
template <typename T, std::enable_if_t<IsLikeVector3<T>::value, std::nullptr_t> = nullptr>
auto isFinite(const T & v) -> bool
{
  return std::isfinite(v.x) and std::isfinite(v.y) and std::isfinite(v.z);
}
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__IS_FINITE_HPP_
