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

#ifndef OPENSCENARIO_INTERPRETER__FUNCTIONAL__EQUAL_TO_HPP_
#define OPENSCENARIO_INTERPRETER__FUNCTIONAL__EQUAL_TO_HPP_

#include <cmath>
#include <functional>
#include <limits>
#include <type_traits>
#include <valarray>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <typename T, typename = void>
struct equal_to : public std::equal_to<T>
{
};

template <typename T>
struct equal_to<T, typename std::enable_if<std::is_floating_point<T>::value>::type>
{
  constexpr auto operator()(const T & lhs, const T & rhs) const noexcept
  {
    return std::abs(lhs - rhs) < std::numeric_limits<typename std::decay<T>::type>::epsilon();
  }
};

template <typename T>
struct equal_to<std::valarray<T>, typename std::enable_if<std::is_floating_point<T>::value>::type>
{
  constexpr auto operator()(
    const std::valarray<T> & lhs, const std::valarray<T> & rhs) const noexcept
  {
    return std::abs(lhs - rhs) < std::numeric_limits<typename std::decay<T>::type>::epsilon();
  }

  constexpr auto operator()(const std::valarray<T> & lhs, const T & rhs) const noexcept
  {
    return std::abs(lhs - rhs) < std::numeric_limits<typename std::decay<T>::type>::epsilon();
  }

  constexpr auto operator()(const T & lhs, const std::valarray<T> & rhs) const noexcept
  {
    return std::abs(lhs - rhs) < std::numeric_limits<typename std::decay<T>::type>::epsilon();
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__FUNCTIONAL__EQUAL_TO_HPP_
