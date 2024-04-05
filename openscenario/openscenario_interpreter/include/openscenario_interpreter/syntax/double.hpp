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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DOUBLE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DOUBLE_HPP_

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Double
{
  using value_type = double;

  value_type data;

  Double() = default;

  Double(value_type);

  explicit Double(const std::string &);

  static auto infinity() noexcept -> Double;

  static auto nan() noexcept -> Double;

  static auto max() noexcept -> Double;

  static auto lowest() noexcept -> Double;

  auto operator=(const value_type & rhs) noexcept -> Double &;

  auto operator+=(const value_type & rhs) noexcept -> Double &;

  auto operator*=(const value_type & rhs) noexcept -> Double &;

  operator value_type() const noexcept;
};

auto operator>>(std::istream &, Double &) -> std::istream &;

auto operator<<(std::ostream &, const Double &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DOUBLE_HPP_
