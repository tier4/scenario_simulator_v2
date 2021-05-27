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
#include <openscenario_interpreter/syntax/vehicle_category.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, VehicleCategory & category)
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                   \
  if (buffer == #IDENTIFIER) {                    \
    category.value = VehicleCategory::IDENTIFIER; \
    return is;                                    \
  }                                               \
  static_assert(true, "")

  BOILERPLATE(bicycle);
  BOILERPLATE(bus);
  BOILERPLATE(car);
  BOILERPLATE(motorbike);
  BOILERPLATE(truck);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                             \
  if (buffer == #IDENTIFIER) {                                              \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(VehicleCategory, buffer); \
  }                                                                         \
  static_assert(true, "")

  BOILERPLATE(semitrailer);
  BOILERPLATE(trailer);
  BOILERPLATE(train);
  BOILERPLATE(tram);
  BOILERPLATE(van);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(VehicleCategory, buffer);
}

std::ostream & operator<<(std::ostream & os, const VehicleCategory & datum)
{
#define BOILERPLATE(NAME)     \
  case VehicleCategory::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(bicycle);
    BOILERPLATE(bus);
    BOILERPLATE(car);
    BOILERPLATE(motorbike);
    BOILERPLATE(semitrailer);
    BOILERPLATE(trailer);
    BOILERPLATE(train);
    BOILERPLATE(tram);
    BOILERPLATE(truck);
    BOILERPLATE(van);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(VehicleCategory, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
