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
constexpr auto isApproximatelyEqualTo(T a, T b) -> bool
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
constexpr auto isEssentiallyEqualTo(T a, T b) -> bool
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::min(std::abs(a), std::abs(b)));
}

template <typename T>
constexpr auto isDefinitelyLessThan(T a, T b) -> bool
{
  return (b - a) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
constexpr auto isDefinitelyGreaterThan(T a, T b) -> bool
{
  return (a - b) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}
}  // namespace arithmetic
}  // namespace math

#endif  // ARITHMETIC__FLOATING_POINT__COMPARISON_HPP_
