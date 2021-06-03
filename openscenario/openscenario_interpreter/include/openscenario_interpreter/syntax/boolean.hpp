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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_

#include <iomanip>
#include <iostream>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/object.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Boolean
{
  using value_type = bool;

  value_type data;

  explicit Boolean() = default;

  explicit constexpr Boolean(value_type value) noexcept : data(value) {}

  explicit Boolean(const std::string & target)
  {
    std::stringstream interpreter;

    if (!(interpreter << target && interpreter >> std::boolalpha >> data)) {
      throw INVALID_NUMERIC_LITERAL_SPECIFIED(Boolean, target);
    }
  }

  auto & operator=(const value_type & rhs) noexcept
  {
    data = rhs;
    return *this;
  }

  constexpr operator value_type() const noexcept { return data; }
};

static_assert(std::is_standard_layout<Boolean>::value, "");

static_assert(std::is_trivial<Boolean>::value, "");

std::istream & operator>>(std::istream &, Boolean &);

std::ostream & operator<<(std::ostream &, const Boolean &);

extern const Element true_v;
extern const Element false_v;

const Element & asBoolean(bool);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
