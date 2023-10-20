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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Maneuver ---------------------------------------------------------------
 *
 *  <xsd:complexType name="Maneuver">
 *    <xsd:sequence>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Event" maxOccurs="unbounded" type="Event"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Maneuver : public Scope, public StoryboardElement
{
  const ParameterDeclarations parameter_declarations;

  explicit Maneuver(const pugi::xml_node &, Scope &);

  auto run() -> void override;

  auto overrideEvents() -> void;

  auto running_events_count() const -> std::size_t;

  friend auto operator<<(nlohmann::json &, const Maneuver &) -> nlohmann::json &;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_
