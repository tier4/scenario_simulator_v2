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

#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <limits>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <regex>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<Double>::value, "");

static_assert(std::is_trivial<Double>::value, "");

Double::Double(value_type value) : data(value) {}

Double::Double(const std::string & s)
try {
  data = boost::lexical_cast<value_type>(s);
} catch (const boost::bad_lexical_cast &) {
  throw INVALID_NUMERIC_LITERAL_SPECIFIED(s);
}

auto Double::infinity() noexcept -> Double
{
  return static_cast<Double>(std::numeric_limits<value_type>::infinity());
}

auto Double::nan() noexcept -> Double
{
  return static_cast<Double>(std::numeric_limits<value_type>::quiet_NaN());
}

auto Double::max() noexcept -> Double
{
  return static_cast<Double>(std::numeric_limits<value_type>::max());
}

auto Double::lowest() noexcept -> Double
{
  return static_cast<Double>(std::numeric_limits<value_type>::lowest());
}

auto Double::operator=(const value_type & rhs) noexcept -> Double &
{
  data = rhs;
  return *this;
}

auto Double::operator+=(const value_type & rhs) noexcept -> Double &
{
  data += rhs;
  return *this;
}

auto Double::operator*=(const value_type & rhs) noexcept -> Double &
{
  data *= rhs;
  return *this;
}

Double::operator value_type() const noexcept { return data; }

auto operator>>(std::istream & is, Double & datum) -> std::istream &
{
  std::string token;

  is >> token;

  static const std::regex infinity{R"([+-]?INF)"};

  std::smatch result;

  if (std::regex_match(token, result, infinity)) {
    datum.data = (result.str(1) == "-" ? -1 : 1) * std::numeric_limits<Double::value_type>::max();
  } else {
    datum.data = boost::lexical_cast<Double::value_type>(token);
  }

  return is;
}

auto operator<<(std::ostream & os, const Double & datum) -> std::ostream &
{
  return os << std::fixed << std::setprecision(30) << datum.data;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
