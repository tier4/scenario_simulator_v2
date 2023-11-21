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

#include <istream>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/entity_action.hpp>
#include <openscenario_interpreter/syntax/entity_object.hpp>
#include <openscenario_interpreter/syntax/external_object_reference.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <ostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto ObjectType::of(const EntityObject & object) -> ObjectType
{
  return ObjectType{apply<ObjectType::value_type>(
    overload(
      [](const Vehicle &) { return ObjectType::vehicle; },
      [](const Pedestrian &) { return ObjectType::pedestrian; },
      [](const MiscObject &) { return ObjectType::miscellaneous; },
      [](const ExternalObjectReference &) { return ObjectType::external; }),
    object)};
}

auto operator>>(std::istream & is, ObjectType & type) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)          \
  if (buffer == #IDENTIFIER) {           \
    type.value = ObjectType::IDENTIFIER; \
    return is;                           \
  }                                      \
  static_assert(true, "")

  BOILERPLATE(vehicle);
  BOILERPLATE(miscellaneous);
  BOILERPLATE(pedestrian);
  BOILERPLATE(external);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(ObjectType, buffer);
}

auto operator<<(std::ostream & os, const ObjectType & type) -> std::ostream &
{
#define BOILERPLATE(IDENTIFIER) \
  case ObjectType::IDENTIFIER:  \
    return os << #IDENTIFIER;

  switch (type) {
    BOILERPLATE(vehicle);
    BOILERPLATE(miscellaneous);
    BOILERPLATE(pedestrian);
    BOILERPLATE(external);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ObjectType, type);
  }

#undef BOILERPLATE
}

}  // namespace syntax
}  // namespace openscenario_interpreter
