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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_

#include <boost/lexical_cast.hpp>
#include <std_msgs/msg/int32.hpp>

#include <string>

namespace open_scenario_interpreter
{
inline namespace syntax
{
struct Integer
  : public std_msgs::msg::Int32
{
  using value_type = decltype(std_msgs::msg::Int32::data);

  explicit Integer(value_type value = {})
  {
    data = value;
  }

  explicit Integer(const std::string & s) try
  {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    std::stringstream ss {};
    ss << "can't treat value \"" << s << "\" as type Integer";
    throw SyntaxError {ss.str()};
  }

  constexpr operator value_type() const noexcept
  {
    return data;
  }

  auto & operator+=(const double & rhs)
  {
    data += rhs;
    return *this;
  }

  auto & operator*=(const double & rhs)
  {
    data *= rhs;
    return *this;
  }
};

std::ostream & operator<<(std::ostream & os, const Integer & rhs)
{
  return os << rhs.data;
}

std::istream & operator>>(std::istream & is, Integer & rhs)
{
  std::string token {};

  is >> token;

  rhs.data = boost::lexical_cast<Integer::value_type>(token);

  return is;
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__INTEGER_HPP_
