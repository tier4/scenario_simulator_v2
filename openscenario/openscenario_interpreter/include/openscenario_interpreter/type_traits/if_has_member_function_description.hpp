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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_MEMBER_FUNCTION_DESCRIPTION_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_MEMBER_FUNCTION_DESCRIPTION_HPP_

#include <openscenario_interpreter/type_traits/has_member_function_description.hpp>

namespace openscenario_interpreter
{
inline namespace type_traits
{
template <typename T, typename = void>
struct IfHasMemberFunctionDescription
{
  static auto invoke(const T &) noexcept -> std::string { return ""; }
};

template <typename T>
struct IfHasMemberFunctionDescription<
  T, typename std::enable_if<HasMemberFunctionDescription<T>::value>::type>
{
  static constexpr decltype(auto) invoke(const T & then) noexcept { return then.description(); }
};
}  // namespace type_traits
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_MEMBER_FUNCTION_DESCRIPTION_HPP_
