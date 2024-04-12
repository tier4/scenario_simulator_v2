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

#include <openscenario_interpreter/syntax/unsigned_integer.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<UnsignedInteger>::value, "");

static_assert(std::is_trivial<UnsignedInteger>::value, "");

UnsignedInteger::UnsignedInteger(value_type value) { data = value; }

UnsignedInteger::UnsignedInteger(const std::string & s)
{
  try {
    data = boost::lexical_cast<value_type>(s);
  } catch (const boost::bad_lexical_cast &) {
    throw INVALID_NUMERIC_LITERAL_SPECIFIED(s);
  }
}

auto UnsignedInteger::operator++() noexcept -> UnsignedInteger &
{
  ++data;
  return *this;
}

auto UnsignedInteger::operator+=(const value_type & rhs) -> UnsignedInteger &
{
  data += rhs;
  return *this;
}

auto UnsignedInteger::operator*=(const value_type & rhs) -> UnsignedInteger &
{
  data *= rhs;
  return *this;
}

UnsignedInteger::operator value_type() const noexcept { return data; }

auto operator>>(std::istream & is, UnsignedInteger & datum) -> std::istream &
{
  return is >> datum.data;
}

auto operator<<(std::ostream & os, const UnsignedInteger & datum) -> std::ostream &
{
  return os << datum.data;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
