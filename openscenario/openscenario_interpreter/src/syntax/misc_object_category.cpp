// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/syntax/misc_object_category.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto operator>>(std::istream & is, MiscObjectCategory & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                   \
  if (buffer == #IDENTIFIER) {                    \
    datum.value = MiscObjectCategory::IDENTIFIER; \
    return is;                                    \
  }                                               \
  static_assert(true, "")

  BOILERPLATE(obstacle);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                                \
  if (buffer == #IDENTIFIER) {                                                 \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(MiscObjectCategory, buffer); \
  }                                                                            \
  static_assert(true, "")

  BOILERPLATE(barrier);
  BOILERPLATE(building);
  BOILERPLATE(crosswalk);
  BOILERPLATE(gantry);
  BOILERPLATE(none);
  BOILERPLATE(parkingSpace);
  BOILERPLATE(patch);
  BOILERPLATE(pole);
  BOILERPLATE(railing);
  BOILERPLATE(roadMark);
  BOILERPLATE(soundBarrier);
  BOILERPLATE(streetLamp);
  BOILERPLATE(trafficIsland);
  BOILERPLATE(tree);
  BOILERPLATE(vegetation);
  BOILERPLATE(wind);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(MiscObjectCategory, buffer);
}

auto operator<<(std::ostream & os, const MiscObjectCategory & datum) -> std::ostream &
{
#define BOILERPLATE(NAME)        \
  case MiscObjectCategory::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(barrier);
    BOILERPLATE(building);
    BOILERPLATE(crosswalk);
    BOILERPLATE(gantry);
    BOILERPLATE(none);
    BOILERPLATE(obstacle);
    BOILERPLATE(parkingSpace);
    BOILERPLATE(patch);
    BOILERPLATE(pole);
    BOILERPLATE(railing);
    BOILERPLATE(roadMark);
    BOILERPLATE(soundBarrier);
    BOILERPLATE(streetLamp);
    BOILERPLATE(trafficIsland);
    BOILERPLATE(tree);
    BOILERPLATE(vegetation);
    BOILERPLATE(wind);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(MiscObjectCategory, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
