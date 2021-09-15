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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/event.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>

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
struct Maneuver : public Scope, public StoryboardElement<Maneuver>, public Elements
{
  const ParameterDeclarations parameter_declarations;

  template <typename Node>
  explicit Maneuver(const Node & node, Scope & outer_scope)
  : Scope(outer_scope.makeChildScope(readAttribute<String>("name", node, outer_scope))),
    parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, localScope()))
  {
    callWithElements(node, "Event", 1, unbounded, [&](auto && node) {
      return push_back(readStoryboardElement<Event>(node, localScope()));
    });
  }

  static constexpr auto ready() noexcept { return true; }

  static constexpr auto start() noexcept -> void {}

  static constexpr auto stopTriggered() noexcept { return false; }

  auto accomplished() const
  {
    // NOTE: A Maneuver's goal is accomplished when all its Events are in the completeState.
    return std::all_of(std::begin(*this), std::end(*this), [](auto && each) {
      return each.template as<Event>().complete();
    });
  }

  using StoryboardElement::evaluate;

  void stop()
  {
    for (auto && each : *this) {
      each.as<Event>().override();
      each.evaluate();
    }
  }

  void run()
  {
    for (auto && each : *this) {
      each.evaluate();
    }
  }
};

nlohmann::json & operator<<(nlohmann::json &, const Maneuver &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_HPP_
