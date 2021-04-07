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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__PAIR_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__PAIR_HPP_

#include <utility>

namespace openscenario_interpreter
{
inline namespace utility
{
auto car = [](auto && pare) noexcept -> decltype(auto) { return std::get<0>(pare); };

auto cdr = [](auto && pare) noexcept -> decltype(auto) { return std::get<1>(pare); };

#define COMPOSE(NAME, F, G)                         \
  template <typename... Ts>                         \
  constexpr decltype(auto) NAME(Ts &&... xs)        \
  {                                                 \
    return F(G(std::forward<decltype(xs)>(xs)...)); \
  }                                                 \
  static_assert(true, "")

COMPOSE(caar, car, car);
COMPOSE(cadr, car, cdr);
COMPOSE(cdar, cdr, car);
COMPOSE(cddr, cdr, cdr);

#undef COMPOSE
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__PAIR_HPP_
