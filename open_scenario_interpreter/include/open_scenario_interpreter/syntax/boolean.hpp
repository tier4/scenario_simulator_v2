// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_

#include <boost/io/ios_state.hpp>
#include <open_scenario_interpreter/object.hpp>

#include <iomanip>
#include <string>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Boolean ==============================================================
 *
 *
 * ======================================================================== */
struct Boolean
{
  using value_type = bool;

  value_type data;

  explicit constexpr Boolean(value_type value = false) noexcept
  : data{value}
  {}

  explicit Boolean(const std::string & target)
  {
    std::stringstream interpreter {};

    if (!(interpreter << target && interpreter >> std::boolalpha >> data)) {
      std::stringstream ss {};
      ss << "can't treat value " << std::quoted(target) << " as type Boolean";
      throw SyntaxError {ss.str()};
    }
  }

  auto & operator=(const value_type & rhs) noexcept
  {
    data = rhs;
    return *this;
  }

  constexpr operator value_type() const noexcept
  {
    return data;
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Boolean & boolean)
{
  boost::io::ios_flags_saver saver {os};
  return os << std::boolalpha << boolean.data;
}

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Boolean & boolean)
{
  boost::io::ios_flags_saver saver {is};
  return is >> std::boolalpha >> boolean.data;
}

static const auto true_v  {make<Boolean>(true)};
static const auto false_v {make<Boolean>(false)};

auto asBoolean(bool value)
{
  return value ? true_v : false_v;
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__BOOLEAN_HPP_
