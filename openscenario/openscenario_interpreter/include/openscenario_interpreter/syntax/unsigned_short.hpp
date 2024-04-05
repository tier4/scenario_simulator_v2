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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_

#include <iostream>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct UnsignedShort
{
  using value_type = std::uint16_t;

  value_type data;

  UnsignedShort() = default;

  explicit UnsignedShort(value_type);

  explicit UnsignedShort(const std::string &);

  auto operator++() noexcept -> UnsignedShort &;

  auto operator+=(const value_type &) -> UnsignedShort &;

  auto operator*=(const value_type &) -> UnsignedShort &;

  operator value_type() const noexcept;
};

auto operator>>(std::istream &, UnsignedShort &) -> std::istream &;

auto operator<<(std::ostream &, const UnsignedShort &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_SHORT_HPP_
