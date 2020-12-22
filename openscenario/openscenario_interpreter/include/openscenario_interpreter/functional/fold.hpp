// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__FUNCTIONAL__FOLD_HPP_
#define OPENSCENARIO_INTERPRETER__FUNCTIONAL__FOLD_HPP_

#include <functional>
#include <utility>

namespace openscenario_interpreter
{
inline namespace functional
{
template
<
  typename F,
  typename T
>
constexpr decltype(auto) fold_left(F &&, T && x)
{
  return std::forward<decltype(x)>(x);
}

template
<
  typename F,
  typename T,
  typename U,
  typename ... Ts
>
constexpr decltype(auto) fold_left(F && f, T && x, U && y, Ts && ... xs)
{
  return fold_left(
    f,
    f(std::forward<decltype(x)>(x), std::forward<decltype(y)>(y)),
    std::forward<decltype(xs)>(xs)...);
}

template
<
  typename F,
  typename T
>
constexpr decltype(auto) fold_right(F &&, T && x)
{
  return std::forward<decltype(x)>(x);
}

template
<
  typename F,
  typename T,
  typename ... Ts
>
constexpr decltype(auto) fold_right(F && f, T && x, Ts && ... xs)
{
  return f(std::forward<decltype(x)>(x), fold_right(f, std::forward<decltype(xs)>(xs)...));
}
}  // namespace functional
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__FUNCTIONAL__FOLD_HPP_
