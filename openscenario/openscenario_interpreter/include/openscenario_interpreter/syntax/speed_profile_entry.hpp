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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_PROFILE_ENTRY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_PROFILE_ENTRY_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedProfileEntry 1.2 --------------------------------------------------
 *
 *  <xsd:complexType name="SpeedProfileEntry">
 *    <xsd:attribute name="speed" type="Double" use="required"/>
 *    <xsd:attribute name="time" type="Double"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedProfileEntry
{
  const Double speed;  // The speed to reach. Unit: [m/s].

  /*
     The time to reach the specified speed. First entry specifies delta from
     start of the action, remaining entries delta from previous entry. If
     omitted, the speed will be reached as soon as possible given the
     performance settings. Unit: [s]. Range: [0..inf[.
  */
  const Double time;

  explicit SpeedProfileEntry(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_PROFILE_ENTRY_HPP_
