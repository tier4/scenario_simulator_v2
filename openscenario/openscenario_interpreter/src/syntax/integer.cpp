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

#include <openscenario_interpreter/syntax/integer.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<Integer>::value, "");

static_assert(std::is_trivial<Integer>::value, "");

Integer::Integer(value_type value) { data = value; }

Integer::Integer(const std::string & s)
{
  try {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    throw INVALID_NUMERIC_LITERAL_SPECIFIED(s);
  }
}

auto Integer::max() noexcept -> Integer
{
  return static_cast<Integer>(std::numeric_limits<value_type>::max());
}

auto Integer::min() noexcept -> Integer
{
  return static_cast<Integer>(std::numeric_limits<value_type>::min());
}

auto Integer::operator+=(const double & rhs) -> Integer &
{
  data += rhs;
  return *this;
}

auto Integer::operator*=(const double & rhs) -> Integer &
{
  data *= rhs;
  return *this;
}

Integer::operator value_type() const noexcept { return data; }

auto operator>>(std::istream & is, Integer & datum) -> std::istream &
{
  std::string token;

  is >> token;

  datum.data = boost::lexical_cast<Integer::value_type>(token);

  return is;
}

auto operator<<(std::ostream & os, const Integer & datum) -> std::ostream &
{
  return os << datum.data;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
