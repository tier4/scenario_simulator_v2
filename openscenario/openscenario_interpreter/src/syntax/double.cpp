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

#include <iomanip>  // std::fixed
#include <limits>   // std::numeric_limits
#include <openscenario_interpreter/syntax/double.hpp>
#include <regex>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, Double & datum)
{
  std::string token;

  is >> token;

  static const std::regex infinity{R"([+-]?INF)"};

  std::smatch result;

#ifndef OPENSCENARIO_INTERPRETER_ALLOW_INFINITY
  constexpr auto upper_bound_value = std::numeric_limits<Double::value_type>::max();
#else
  constexpr auto upper_bound_value = std::numeric_limits<Double::value_type>::infinity();
#endif

  if (std::regex_match(token, result, infinity)) {
    datum.data = (result.str(1) == "-" ? -1 : 1) * upper_bound_value;
  } else {
    datum.data = boost::lexical_cast<Double::value_type>(token);
  }

  return is;
}

std::ostream & operator<<(std::ostream & os, const Double & datum)
{
  return os << std::fixed << datum.data;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
