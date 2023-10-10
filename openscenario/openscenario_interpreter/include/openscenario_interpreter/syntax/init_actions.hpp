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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- InitActions ------------------------------------------------------------
 *
 *  <xsd:complexType name="InitActions">
 *    <xsd:sequence>
 *      <xsd:element name="GlobalAction" type="GlobalAction" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="UserDefinedAction" type="UserDefinedAction" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Private" minOccurs="0" maxOccurs="unbounded" type="Private"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct InitActions : public StoryboardElement
{
  explicit InitActions(const pugi::xml_node &, Scope &);

  auto evaluate() -> Object override;

  auto accomplished() const -> bool override;

  auto endsImmediately() const -> bool;

  auto run() -> void override;

  auto runInstantaneousActions() -> void;

  auto runNonInstantaneousActions() -> void;

  auto start() -> void override;

  auto startInstantaneousActions() -> void;

  auto startNonInstantaneousActions() -> void;

  Elements global_actions;
  Elements user_defined_actions;
  Elements privates;

  friend auto operator<<(nlohmann::json &, const InitActions &) -> nlohmann::json &;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_
