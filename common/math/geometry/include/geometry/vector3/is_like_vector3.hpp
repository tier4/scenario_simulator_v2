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

#ifndef GEOMETRY__VECTOR3__IS_LIKE_VECTOR3_HPP_
#define GEOMETRY__VECTOR3__IS_LIKE_VECTOR3_HPP_

#include <type_traits>
#include <utility>

namespace math
{
namespace geometry
{
template <typename T, typename = void>
struct HasMemberW : std::false_type
{
};

template <typename T>
struct HasMemberW<T, std::void_t<decltype(std::declval<T>().w)>> : std::true_type
{
};

template <typename T, typename = void>
struct IsLikeVector3 : public std::false_type
{
};

template <typename T>
struct IsLikeVector3<
  T, std::void_t<
       decltype(std::declval<T>().x), decltype(std::declval<T>().y), decltype(std::declval<T>().z),
       std::enable_if_t<!HasMemberW<T>::value>>> : public std::true_type
{
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__VECTOR3__IS_LIKE_VECTOR3_HPP_
