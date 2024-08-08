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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter/error.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Integer
{
  using value_type = std::int64_t;

  value_type data;

  Integer() = default;

  Integer(value_type);

  explicit Integer(const std::string &);

  static auto max() noexcept -> Integer;

  static auto min() noexcept -> Integer;

  auto operator+=(const double &) -> Integer &;

  auto operator*=(const double &) -> Integer &;

  operator value_type() const noexcept;
};

auto operator>>(std::istream &, Integer &) -> std::istream &;

auto operator<<(std::ostream &, const Integer &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_
