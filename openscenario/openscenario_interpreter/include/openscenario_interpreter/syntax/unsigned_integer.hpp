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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_INTEGER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_INTEGER_HPP_

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter/error.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct UnsignedInteger
{
  using value_type = std::uint64_t;

  value_type data;

  UnsignedInteger() = default;

  UnsignedInteger(value_type);

  explicit UnsignedInteger(const std::string &);

  auto operator++() noexcept -> UnsignedInteger &;

  auto operator+=(const value_type &) -> UnsignedInteger &;

  auto operator*=(const value_type &) -> UnsignedInteger &;

  operator value_type() const noexcept;
};

std::istream & operator>>(std::istream &, UnsignedInteger &);

std::ostream & operator<<(std::ostream &, const UnsignedInteger &);

using UnsignedInt = UnsignedInteger;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__UNSIGNED_INTEGER_HPP_
