

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
#include <openscenario_interpreter/syntax/wetness.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<Wetness>::value, "");

static_assert(std::is_trivial<Wetness>::value, "");

auto operator>>(std::istream & is, Wetness & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)        \
  if (buffer == #IDENTIFIER) {         \
    datum.value = Wetness::IDENTIFIER; \
    return is;                         \
  }                                    \
  static_assert(true, "")

  BOILERPLATE(dry);
  BOILERPLATE(moist);
  BOILERPLATE(wetWithPuddles);
  BOILERPLATE(lowFlooded);
  BOILERPLATE(highFlooded);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Wetness, buffer);
}

auto operator<<(std::ostream & os, const Wetness & datum) -> std::ostream &
{
  switch (datum) {
#define BOILERPLATE(ID) \
  case Wetness::ID:     \
    return os << #ID;

    BOILERPLATE(dry);
    BOILERPLATE(moist);
    BOILERPLATE(wetWithPuddles);
    BOILERPLATE(lowFlooded);
    BOILERPLATE(highFlooded);

#undef BOILERPLATE

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Wetness, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
