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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__MUST_BE_DEFAULT_CONSTRUCTIBLE_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__MUST_BE_DEFAULT_CONSTRUCTIBLE_HPP_

#include <openscenario_interpreter/error.hpp>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace type_traits
{
template <typename T, typename = void>
struct MustBeDefaultConstructible
{
  template <typename Error>
  static T makeItOrThrow(Error && error)
  {
    throw error;
  }
};

template <typename T>
struct MustBeDefaultConstructible<
  T, typename std::enable_if<std::is_default_constructible<T>::value>::type>
{
  template <typename... Ts>
  static T makeItOrThrow(Ts &&...)
  {
    return T();
  }
};
}  // namespace type_traits
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__TYPE_TRAITS__MUST_BE_DEFAULT_CONSTRUCTIBLE_HPP_
