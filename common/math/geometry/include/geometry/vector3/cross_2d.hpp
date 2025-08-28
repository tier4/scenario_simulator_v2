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

#ifndef GEOMETRY__CROSS_2D__HYPOT_HPP_
#define GEOMETRY__CROSS_2D__HYPOT_HPP_

#include <cmath>
#include <geometry/vector3/is_like_vector3.hpp>

namespace math
{
namespace geometry
{
template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<IsLikeVector3<T>, IsLikeVector3<U>>, std::nullptr_t> =
    nullptr>
auto cross_2d(const T & a, const U & b)
{
  return a.x * b.y - a.y * b.x;
}
}  // namespace geometry
}  // namespace math

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_MATH_UTILS_HPP_
