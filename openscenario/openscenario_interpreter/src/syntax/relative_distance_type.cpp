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
#include <openscenario_interpreter/syntax/relative_distance_type.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, RelativeDistanceType & datum)
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                     \
  if (buffer == #IDENTIFIER) {                      \
    datum.value = RelativeDistanceType::IDENTIFIER; \
    return is;                                      \
  }                                                 \
  static_assert(true, "")

  BOILERPLATE(longitudinal);
  BOILERPLATE(lateral);
  BOILERPLATE(euclidianDistance);

#undef BOILERPLATE

  // NOTE: cartesianDistance is deprecated (since OpenSCENARIO 1.1)
  if (buffer == "cartesianDistance") {
    datum.value = RelativeDistanceType::euclidianDistance;
    return is;
  }

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(RelativeDistanceType, buffer);
}

std::ostream & operator<<(std::ostream & os, const RelativeDistanceType & datum)
{
  switch (datum) {
#define BOILERPLATE(ID)          \
  case RelativeDistanceType::ID: \
    return os << #ID;

    BOILERPLATE(longitudinal);
    BOILERPLATE(lateral);
    BOILERPLATE(euclidianDistance);

#undef BOILERPLATE

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(RelativeDistanceType, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
