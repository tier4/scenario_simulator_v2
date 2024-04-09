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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PRIORITY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PRIORITY_HPP_

#include <openscenario_interpreter/object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- PRIORITY ---------------------------------------------------------------
 *
 *  <xsd:simpleType name="Priority">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="overwrite"/>
 *          <xsd:enumeration value="skip"/>
 *          <xsd:enumeration value="parallel"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct Priority
{
  enum value_type {

    /*
       If a starting event has priority Overwrite, all events in running state,
       within the same scope (maneuver) as the starting event, should be issued
       a stop command (stop transition).
    */
    overwrite,

    /*
       If a starting event has priority Skip, then it will not be run if there
       is any other event in the same scope (maneuver) in the running state.
    */
    skip,

    /*
       Execute in parallel to other events.
    */
    parallel,
  } value;

  Priority() = default;

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, Priority &) -> std::istream &;

auto operator<<(std::ostream &, const Priority &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRIORITY_HPP_
