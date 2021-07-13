// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__OVERLOAD_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__OVERLOAD_HPP_

#include <type_traits>
#include <utility>

namespace openscenario_interpreter
{
inline namespace utility
{
template <typename...>
struct overloaded;

template <typename T>
struct overloaded<T> : public T
{
  overloaded(T && x) : T(std::forward<decltype(x)>(x)) {}

  using T::operator();
};

template <typename T, typename U, typename... Ts>
struct overloaded<T, U, Ts...> : public T, public overloaded<U, Ts...>
{
  overloaded(T && x, U && y, Ts &&... xs)
  : T(std::forward<decltype(x)>(x)),
    overloaded<U, Ts...>(std::forward<decltype(y)>(y), std::forward<decltype(xs)>(xs)...)
  {
  }

  using T::operator();

  using overloaded<U, Ts...>::operator();
};

template <typename... Ts>
constexpr auto overload(Ts &&... xs) ->
  typename utility::overloaded<typename std::decay<Ts>::type...>
{
  return {std::forward<decltype(xs)>(xs)...};
}
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__OVERLOAD_HPP_
