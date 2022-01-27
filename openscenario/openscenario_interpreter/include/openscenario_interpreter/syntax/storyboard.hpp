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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/init.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Storyboard -------------------------------------------------------------
 *
 *  <xsd:complexType name="Storyboard">
 *    <xsd:sequence>
 *      <xsd:element name="Init" type="Init"/>
 *      <xsd:element name="Story" maxOccurs="unbounded" type="Story"/>
 *      <xsd:element name="StopTrigger" type="Trigger"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Storyboard : public Scope, public StoryboardElement
{
  Init init;

  Elements stories;

  Trigger stop_trigger;

  bool engaged = false;

  explicit Storyboard(const pugi::xml_node &, Scope &);

  auto accomplished() const -> bool override;

  auto elements() -> Elements & override;

  auto ready() noexcept -> bool override;

  auto run() -> void override;

  auto start() -> void override;

  auto stop() -> void override;

  auto stopTriggered() -> bool override;
};

auto operator<<(nlohmann::json &, const Storyboard &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
