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

#ifndef OPENSCENARIO_INTERPRETER__OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__OBJECT_HPP_

#include <list>
#include <openscenario_interpreter/expression.hpp>
#include <type_traits>
#include <utility>

namespace openscenario_interpreter
{
using Element = Pointer<Expression>;

using ComplexType = Element;

using Group = Element;

using Elements = std::list<Element>;

template <typename... Ts>
using IsOptionalElement = std::is_default_constructible<Ts...>;

template <typename T, typename... Ts>
constexpr auto make(Ts &&... xs) -> decltype(auto)
{
  return Element::bind<T>(std::forward<decltype(xs)>(xs)...);
}

template <typename T>
constexpr auto make(T && x) -> decltype(auto)
{
  return Element::bind<typename std::decay<decltype(x)>::type>(std::forward<decltype(x)>(x));
}

extern const Element unspecified;

struct Unspecified
{
  decltype(auto) evaluate() const noexcept
  {
    return unspecified;  // Self-evaluating.
  }
};

auto operator<<(std::ostream &, const Unspecified &) -> std::ostream &;

#define CASE(TYPE)                                                                   \
  {                                                                                  \
    typeid(TYPE), [](Function && function, auto && datum, Ts &&... xs) {             \
      return function(datum.template as<TYPE>(), std::forward<decltype(xs)>(xs)...); \
    }                                                                                \
  }

#define DEFINE_LAZY_VISITOR(TYPE, ...)                                                         \
  template <typename Result, typename Function, typename... Ts>                                \
  auto apply(Function && function, TYPE & datum, Ts &&... xs)                                  \
  {                                                                                            \
    try {                                                                                      \
      static const std::unordered_map<                                                         \
        std::type_index, std::function<Result(Function &&, TYPE &, Ts &&...)>>                 \
        overloads{__VA_ARGS__};                                                                \
      return overloads.at(datum.type())(                                                       \
        std::forward<decltype(function)>(function), datum, std::forward<decltype(xs)>(xs)...); \
    } catch (const std::out_of_range &) {                                                      \
      throw UNSUPPORTED_SETTING_DETECTED(TYPE, makeTypename(datum.type().name()));             \
    }                                                                                          \
  }                                                                                            \
  static_assert(true, "")
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OBJECT_HPP_
