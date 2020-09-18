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

#ifndef SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_
#define SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_

#include <scenario_runner/object.hpp>

#include <string>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== Command ==============================================================
 *
 * TODO
 *
 * ======================================================================== */
struct Command
{
  enum value_type
  {
    debugAccomplishment,
    exitFailure,
    exitSuccess,
    print,
  }
  value;

  explicit constexpr Command(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }

  decltype(auto) operator=(const value_type & rhs)
  {
    value = rhs;
    return *this;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Command & command)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    command = Command::IDENTIFIER; \
    return is; \
  }

  BOILERPLATE(debugAccomplishment);
  BOILERPLATE(exitFailure);
  BOILERPLATE(exitSuccess);
  BOILERPLATE(print);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type Command";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Command & command)
{
  switch (command) {
    #define BOILERPLATE(NAME) case Command::NAME: return os << #NAME;

    BOILERPLATE(debugAccomplishment);
    BOILERPLATE(exitFailure);
    BOILERPLATE(exitSuccess);
    BOILERPLATE(print);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class Command holds unexpected value " <<
        static_cast<Command::value_type>(command);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__COMMAND_HPP_
