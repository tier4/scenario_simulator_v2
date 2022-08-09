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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__COMMAND_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__COMMAND_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Command
{
  enum value_type {
    exitFailure,
    exitSuccess,
    nop,
    print,
  } value;

  explicit Command() = default;

  constexpr operator value_type() const noexcept { return value; }

  auto operator=(const value_type &) -> Command &;
};

auto operator>>(std::istream &, Command &) -> std::istream &;

auto operator<<(std::ostream &, const Command &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__COMMAND_HPP_
