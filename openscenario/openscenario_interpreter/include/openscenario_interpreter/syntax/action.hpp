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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/global_action.hpp>
#include <openscenario_interpreter/syntax/private_action.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/user_defined_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Action -----------------------------------------------------------------
 *
 *  <xsd:complexType name="Action">
 *    <xsd:choice>
 *      <xsd:element name="GlobalAction" type="GlobalAction"/>
 *      <xsd:element name="UserDefinedAction" type="UserDefinedAction"/>
 *      <xsd:element name="PrivateAction" type="PrivateAction"/>
 *    </xsd:choice>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Action : public Scope, public ComplexType, public StoryboardElement
{
  bool overridden = false;

  explicit Action(const pugi::xml_node &, Scope &);

  using StoryboardElement::evaluate;

  /* -------------------------------------------------------------------------
   *
   *  An Action's goal is a function of the Action type and cannot be
   *  generalized. Accomplishing an Action's goal will involve meeting some
   *  arbitrary prerequisites related with the Action type (for example, a
   *  SpeedAction accomplishes its goal when the considered Entity is
   *  travelling at the prescribed speed). If an Action is acting on an
   *  EntitySelection, all instances of Entity within the selection have to
   *  complete in order to reach the completeState of the Action.
   *
   * ---------------------------------------------------------------------- */
  auto accomplished() const -> bool override;

  auto elements() -> Elements & override;

  auto ready() -> bool override;

  auto run() -> void override;

  auto start() -> void override;

  auto stop() -> void override;

  auto stopTriggered() noexcept -> bool override;
};

auto operator<<(nlohmann::json &, const Action &) -> nlohmann::json &;

DEFINE_LAZY_VISITOR(
  Action,                   //
  CASE(GlobalAction),       //
  CASE(UserDefinedAction),  //
  CASE(PrivateAction),      //
);

DEFINE_LAZY_VISITOR(
  const Action,             //
  CASE(GlobalAction),       //
  CASE(UserDefinedAction),  //
  CASE(PrivateAction),      //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_
