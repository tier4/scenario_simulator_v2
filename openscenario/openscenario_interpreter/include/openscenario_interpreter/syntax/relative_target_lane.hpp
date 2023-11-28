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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_LANE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_LANE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <pugixml.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeTargetLane -----------------------------------------------------
 *
 *  <xsd:complexType name="RelativeTargetLane">
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="value" type="Int" use="required"/>
 *  </xsd:complexType>*
 *
 * -------------------------------------------------------------------------- */
struct RelativeTargetLane
{
  const Entity entity_ref;

  const Integer value;

  explicit RelativeTargetLane(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator::lane_change::Direction() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_LANE_HPP_
