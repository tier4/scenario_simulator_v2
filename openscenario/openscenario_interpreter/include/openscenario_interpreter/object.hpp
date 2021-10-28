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
#include <typeindex>
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

template <typename T, typename Function>
auto invoke_dispatch(Element & datum, Function && function)
{
  if (datum.is_also<T>()) {
    return function(datum.as<T>());
  }
  throw UNSUPPORTED_SETTING_DETECTED(TYPE, makeTypename(datum.type().name()));
}

template <typename T, typename... Ts, typename Function>
auto invoke_dispatch(Element & datum, Function && function)
{
  if (datum.is_also<T>()) {
    return function(datum.as<T>());
  }

  return invoke_dispatch<Ts...>(datum, std::forward<Function>(function));
}
}  // namespace openscenario_interpreter

#define DEFINE_LAZY_VISITOR(TYPE, ...)                                                        \
  template <typename Result, typename Function, typename... Args>                             \
  auto apply(Function && function, TYPE & datum, Args &&... args)                             \
  {                                                                                           \
    return ::openscenario_interpreter::invoke_dispatch<__VA_ARGS__>(datum, [&](auto && arg) { \
      return function(std::forward<decltype(arg)>(arg), std::forward<Args>(args)...);         \
    });                                                                                       \
  }

#endif  // OPENSCENARIO_INTERPRETER__OBJECT_HPP_
