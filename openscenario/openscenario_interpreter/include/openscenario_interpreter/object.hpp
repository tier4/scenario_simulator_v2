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
using Object = Pointer<Expression>;

template <typename T>
struct is
{
  auto operator()(const Object & object) const { return object.is<T>(); }
};

template <>
struct is<Object>
{
  auto operator()(const Object &) const noexcept { return true; }
};

template <typename T>
struct is_also
{
  auto operator()(const Object & object) const { return object.is_also<T>(); }
};

template <>
struct is_also<Object>
{
  auto operator()(const Object &) const noexcept { return true; }
};

using ComplexType = Object;

using Group = Object;

using Elements = std::list<Object>;

template <typename T, typename... Ts>
constexpr auto make(Ts &&... xs) -> decltype(auto)
{
  return Object::bind<T>(std::forward<decltype(xs)>(xs)...);
}

template <typename T>
constexpr auto make(T && x) -> decltype(auto)
{
  return Object::bind<typename std::decay<decltype(x)>::type>(std::forward<decltype(x)>(x));
}

extern const Object unspecified;

struct Unspecified
{
  auto evaluate() const noexcept -> decltype(auto)
  {
    return unspecified;  // Self-evaluating.
  }
};

auto operator<<(std::ostream &, const Unspecified &) -> std::ostream &;
}  // namespace openscenario_interpreter

#define CASE(TYPE)                                                               \
  {                                                                              \
    [](auto & datum) { return datum.template is_also<TYPE>(); },                 \
      [&](auto & datum, auto &&... args) {                                       \
        return function(datum.template as<TYPE>(), std::forward<Args>(args)...); \
      }                                                                          \
  }

#define DEFINE_LAZY_VISITOR(TYPE, ...)                                                             \
  template <typename Result, typename Function, typename... Args>                                  \
  Result apply(Function && function, TYPE & datum, Args &&... args)                                \
  {                                                                                                \
    std::vector<std::pair<std::function<bool(TYPE &)>, std::function<Result(TYPE &, Args &&...)>>> \
      dispatcher{{__VA_ARGS__}};                                                                   \
    for (auto & p : dispatcher) {                                                                  \
      if (p.first(datum)) {                                                                        \
        return p.second(datum, std::forward<Args>(args)...);                                       \
      }                                                                                            \
    }                                                                                              \
    throw UNSUPPORTED_SETTING_DETECTED(TYPE, makeTypename(datum.type().name()));                   \
  }                                                                                                \
  static_assert(true, "")

#endif  // OPENSCENARIO_INTERPRETER__OBJECT_HPP_
