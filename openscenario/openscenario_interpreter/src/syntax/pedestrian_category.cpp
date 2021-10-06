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

#include <iostream>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/pedestrian_category.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto operator>>(std::istream & is, PedestrianCategory & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                   \
  if (buffer == #IDENTIFIER) {                    \
    datum.value = PedestrianCategory::IDENTIFIER; \
    return is;                                    \
  }                                               \
  static_assert(true, "")

  BOILERPLATE(pedestrian);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                                \
  if (buffer == #IDENTIFIER) {                                                 \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(PedestrianCategory, buffer); \
  }                                                                            \
  static_assert(true, "")

  BOILERPLATE(wheelchair);
  BOILERPLATE(animal);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(PedestrianCategory, buffer);
}

auto operator<<(std::ostream & os, const PedestrianCategory & datum) -> std::ostream &
{
  switch (datum) {
#define BOILERPLATE(NAME)        \
  case PedestrianCategory::NAME: \
    return os << #NAME;

    BOILERPLATE(pedestrian);
    BOILERPLATE(wheelchair);
    BOILERPLATE(animal);

#undef BOILERPLATE

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(PedestrianCategory, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
