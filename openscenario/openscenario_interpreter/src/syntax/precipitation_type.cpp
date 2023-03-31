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
#include <openscenario_interpreter/syntax/precipitation_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<PrecipitationType>::value, "");

static_assert(std::is_trivial<PrecipitationType>::value, "");

auto operator>>(std::istream & is, PrecipitationType & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                  \
  if (buffer == #IDENTIFIER) {                   \
    datum.value = PrecipitationType::IDENTIFIER; \
    return is;                                   \
  }                                              \
  static_assert(true, "")

  BOILERPLATE(dry);
  BOILERPLATE(rain);
  BOILERPLATE(snow);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(PrecipitationType, buffer);
}

auto operator<<(std::ostream & os, const PrecipitationType & datum) -> std::ostream &
{
  switch (datum) {
#define BOILERPLATE(ID)       \
  case PrecipitationType::ID: \
    return os << #ID;

    BOILERPLATE(dry);
    BOILERPLATE(rain);
    BOILERPLATE(snow);

#undef BOILERPLATE

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(PrecipitationType, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
