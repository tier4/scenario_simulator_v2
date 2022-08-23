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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/actors.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ManeuverGroup ----------------------------------------------------------
 *
 *  <xsd:complexType name="ManeuverGroup">
 *    <xsd:sequence>
 *      <xsd:element name="Actors" type="Actors"/>
 *      <xsd:element name="CatalogReference" type="CatalogReference" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Maneuver" type="Maneuver" minOccurs="0" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="maximumExecutionCount" type="UnsignedInt" use="required"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ManeuverGroup : public Scope, public StoryboardElement
{
  const Actors actors;

  explicit ManeuverGroup(const pugi::xml_node &, Scope &);

  auto start() -> void override;
};

auto operator<<(nlohmann::json &, const ManeuverGroup &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_
