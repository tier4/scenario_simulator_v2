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

#include <boost/io/ios_state.hpp>
#include <iomanip>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/object.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Boolean ----------------------------------------------------------------
 *
 *
 * -------------------------------------------------------------------------- */
struct Boolean
{
  using value_type = bool;

  value_type data;

  explicit constexpr Boolean(value_type value = false) noexcept : data(value) {}

  explicit Boolean(const std::string & target)
  {
    std::stringstream interpreter{};

    if (!(interpreter << target && interpreter >> std::boolalpha >> data)) {
      std::stringstream ss{};
      ss << "can't treat value " << std::quoted(target) << " as type Boolean";
      throw SyntaxError(ss.str());
    }
  }

  auto & operator=(const value_type & rhs) noexcept
  {
    data = rhs;
    return *this;
  }

  constexpr operator value_type() const noexcept { return data; }
};

std::istream & operator>>(std::istream &, Boolean &);
std::ostream & operator<<(std::ostream &, const Boolean &);

extern const Element true_v;
extern const Element false_v;

const Element & asBoolean(bool);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
