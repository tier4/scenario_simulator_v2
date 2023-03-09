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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<ParameterType>::value, "");

static_assert(std::is_trivial<ParameterType>::value, "");

auto operator>>(std::istream & is, ParameterType & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(NAME, IDENTIFIER)        \
  if (buffer == NAME) {                      \
    datum.value = ParameterType::IDENTIFIER; \
    return is;                               \
  }                                          \
  static_assert(true, "")

  BOILERPLATE("integer", INTEGER);
  BOILERPLATE("double", DOUBLE);
  BOILERPLATE("string", STRING);
  BOILERPLATE("unsignedInt", UNSIGNED_INT);
  BOILERPLATE("unsignedShort", UNSIGNED_SHORT);
  BOILERPLATE("boolean", BOOLEAN);
  BOILERPLATE("dateTime", DATE_TIME);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(ParameterType, buffer);
}

auto operator<<(std::ostream & os, const ParameterType & datum) -> std::ostream &
{
#define BOILERPLATE(NAME, ID) \
  case ParameterType::ID:     \
    return os << NAME;

  switch (datum) {
    BOILERPLATE("integer", INTEGER);
    BOILERPLATE("double", DOUBLE);
    BOILERPLATE("string", STRING);
    BOILERPLATE("unsignedInt", UNSIGNED_INT);
    BOILERPLATE("unsignedShort", UNSIGNED_SHORT);
    BOILERPLATE("boolean", BOOLEAN);
    BOILERPLATE("dateTime", DATE_TIME);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ParameterType, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
