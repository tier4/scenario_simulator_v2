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

#ifndef GEOMETRY__QUATERNION__IS_LIKE_QUATERNION_HPP_
#define GEOMETRY__QUATERNION__IS_LIKE_QUATERNION_HPP_

#include <type_traits>
#include <utility>

namespace math
{
namespace geometry
{
template <typename T, typename = void>
struct IsLikeQuaternion : std::false_type
{
};

template <typename T>
struct IsLikeQuaternion<
  T, std::void_t<
       decltype(std::declval<T>().x), decltype(std::declval<T>().y), decltype(std::declval<T>().z),
       decltype(std::declval<T>().w)>> : std::true_type
{
};

}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__QUATERNION__IS_LIKE_QUATERNION_HPP_
