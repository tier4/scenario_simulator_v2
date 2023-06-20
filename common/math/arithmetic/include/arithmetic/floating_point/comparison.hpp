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

#ifndef ARITHMETIC__FLOATING_POINT__COMPARISON_HPP_
#define ARITHMETIC__FLOATING_POINT__COMPARISON_HPP_

#include <algorithm>
#include <limits>

namespace math
{
namespace arithmetic
{
template <typename T>
auto isApproximatelyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto isEssentiallyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::min(std::abs(a), std::abs(b)));
}

template <typename T, typename... Ts>
auto isDefinitelyLessThan(T a, T b, Ts... xs)
{
  auto compare = [](T a, T b) {
    return (b - a) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
  };

  if constexpr (0 < sizeof...(Ts)) {
    return compare(a, b) and compare(b, xs...);
  } else {
    return compare(a, b);
  }
}

template <typename T>
auto isDefinitelyGreaterThan(T a, T b)
{
  return (a - b) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}
}  // namespace arithmetic
}  // namespace math

#endif  // ARITHMETIC__FLOATING_POINT__COMPARISON_HPP_
