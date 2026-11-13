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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_

#include <boost/json.hpp>
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

  auto accomplished() const -> bool override;

  auto endsImmediately() const -> bool;

  auto run() -> void override;

  auto start() -> void override;

  auto stop() -> void override;

  friend auto operator<<(boost::json::object &, const Action &) -> boost::json::object &;
};

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
