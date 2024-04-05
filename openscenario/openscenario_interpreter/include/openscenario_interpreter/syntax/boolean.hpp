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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_

#include <iostream>
#include <openscenario_interpreter/object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Boolean
{
  using value_type = bool;

  value_type data;

  Boolean() = default;

  constexpr Boolean(value_type value) noexcept : data(value) {}

  explicit Boolean(const std::string &);

  auto operator=(const value_type & rhs) noexcept -> Boolean &;

  constexpr operator value_type() const noexcept { return data; }
};

auto operator>>(std::istream &, Boolean &) -> std::istream &;

auto operator<<(std::ostream &, const Boolean &) -> std::ostream &;

extern const Object true_v;

extern const Object false_v;

auto asBoolean(bool) -> const Object &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
