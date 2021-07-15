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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/actors.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>

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
struct ManeuverGroup : public StoryboardElement<ManeuverGroup>, public Elements
{
  const String name;

  Scope inner_scope;

  const Actors actors;

  template <typename Node>
  explicit ManeuverGroup(const Node & node, Scope & outer_scope)
  : StoryboardElement(readAttribute<UnsignedInteger>(
      "maximumExecutionCount", node, outer_scope, UnsignedInteger())),
    name(readAttribute<String>("name", node, outer_scope)),
    inner_scope(outer_scope),
    actors(readElement<Actors>("Actors", node, inner_scope))
  {
    callWithElements(node, "CatalogReference", 0, unbounded, [&](auto && node) {
      throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name());
      return unspecified;
    });

    callWithElements(node, "Maneuver", 0, unbounded, [&](auto && node) {
      return push_back(readStoryboardElement<Maneuver>(node, inner_scope));
    });
  }

  static constexpr auto ready() noexcept { return true; }

  static constexpr auto stopTriggered() noexcept { return false; }

  /* ---------------------------------------------------------------------------
   *
   *  A ManeuverGroup's goal is accomplished when all its Maneuvers are in the
   *  completeState.
   *
   * ------------------------------------------------------------------------ */
  auto accomplished() const
  {
    return std::all_of(std::begin(*this), std::end(*this), [&](auto && each) {
      return each.template as<Maneuver>().complete();
    });
  }

  using StoryboardElement::evaluate;

  void start()
  {
    for (auto && each : *this) {
      each.as<Maneuver>().changeStateIf(true, standby_state);
    }
  }

  void stop()
  {
    for (auto && each : *this) {
      each.as<Maneuver>().override();
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

nlohmann::json & operator<<(nlohmann::json &, const ManeuverGroup &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MANEUVER_GROUP_HPP_
