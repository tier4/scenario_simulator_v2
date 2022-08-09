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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__DEMANGLE_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__DEMANGLE_HPP_

#include <string>
#include <typeinfo>

namespace openscenario_interpreter
{
inline namespace utility
{
auto demangle(const char * name) -> std::string;

auto demangle(const std::type_info &) -> std::string;

template <typename... Ts>
auto makeTypename(Ts &&... xs)
{
  const auto name = demangle(std::forward<decltype(xs)>(xs)...);

  return name.substr(name.find_last_of(':') + 1);
}
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__DEMANGLE_HPP_
