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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/priority.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Event ------------------------------------------------------------------
 *
 *  <xsd:complexType name="Event">
 *    <xsd:sequence>
 *      <xsd:element name="Action" maxOccurs="unbounded" type="Action"/>
 *      <xsd:element name="StartTrigger" type="Trigger"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="priority" type="Priority" use="required"/>
 *    <xsd:attribute name="maximumExecutionCount" type="UnsignedInt"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Event : private Scope, public StoryboardElement
{
  using Scope::name;

  const Priority priority;  // Priority of each event.

  explicit Event(const pugi::xml_node &, Scope &, Maneuver &);

  auto start() -> void override;

  auto evaluate() -> Object override;

  friend auto operator<<(nlohmann::json &, const Event &) -> nlohmann::json &;

private:
  Maneuver & parent_maneuver;
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
