// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__OBJECT_HPP_
#define SCENARIO_RUNNER__OBJECT_HPP_

#include <scenario_runner/expression.hpp>
#include <utility>
#include <vector>

namespace scenario_runner
{
using Element = Pointer<Expression>;

using Elements = std::vector<Element>;

// static const Element unit {nullptr};

template<typename T, typename ... Ts>
inline constexpr decltype(auto) make(Ts && ... xs)
{
  return Element::bind<T>(std::forward<decltype(xs)>(xs)...);
}

template<typename T>
inline constexpr decltype(auto) make(T && x)
{
  return Element::bind<typename std::decay<decltype(x)>::type>(std::forward<decltype(x)>(x));
}

extern const Element unspecified;

struct Unspecified
{
  decltype(auto) evaluate() const noexcept
  {
    return unspecified;
  }
};

std::ostream & operator<<(std::ostream &, const Unspecified &);
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__OBJECT_HPP_
