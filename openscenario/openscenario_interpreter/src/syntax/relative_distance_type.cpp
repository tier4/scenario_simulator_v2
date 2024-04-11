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

#include <iostream>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/relative_distance_type.hpp>
#include <string>
#include <type_traits>

// Ignore spell miss due to OpenSCENARIO standard.
// cspell: ignore euclidian

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<RelativeDistanceType>::value, "");

static_assert(std::is_trivial<RelativeDistanceType>::value, "");

auto operator>>(std::istream & is, RelativeDistanceType & datum) -> std::istream &
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

auto operator<<(std::ostream & os, const RelativeDistanceType & datum) -> std::ostream &
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
