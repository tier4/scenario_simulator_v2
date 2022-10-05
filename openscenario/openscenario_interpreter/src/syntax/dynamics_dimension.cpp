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
#include <openscenario_interpreter/syntax/dynamics_dimension.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<DynamicsDimension>::value);

static_assert(std::is_trivial<DynamicsDimension>::value);

auto operator>>(std::istream & is, DynamicsDimension & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                  \
  if (buffer == #IDENTIFIER) {                   \
    datum.value = DynamicsDimension::IDENTIFIER; \
    return is;                                   \
  }                                              \
  static_assert(true, "")

  BOILERPLATE(rate);
  BOILERPLATE(time);
  BOILERPLATE(distance);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                               \
  if (buffer == #IDENTIFIER) {                                                \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(DynamicsDimension, buffer); \
  }                                                                           \
  static_assert(true, "")

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(DynamicsDimension, buffer);
}

auto operator<<(std::ostream & os, const DynamicsDimension & datum) -> std::ostream &
{
#define BOILERPLATE(NAME)       \
  case DynamicsDimension::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(rate);
    BOILERPLATE(time);
    BOILERPLATE(distance);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(DynamicsDimension, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
