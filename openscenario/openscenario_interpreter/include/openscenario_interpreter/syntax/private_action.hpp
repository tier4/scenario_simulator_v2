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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/controller_action.hpp>
#include <openscenario_interpreter/syntax/lateral_action.hpp>
#include <openscenario_interpreter/syntax/longitudinal_action.hpp>
#include <openscenario_interpreter/syntax/routing_action.hpp>
#include <openscenario_interpreter/syntax/teleport_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- PrivateAction ----------------------------------------------------------
 *
 *  <xsd:complexType name="PrivateAction">
 *    <xsd:choice>
 *      <xsd:element name="LongitudinalAction" type="LongitudinalAction"/>
 *      <xsd:element name="LateralAction" type="LateralAction"/>
 *      <xsd:element name="VisibilityAction" type="VisibilityAction"/>
 *      <xsd:element name="SynchronizeAction" type="SynchronizeAction"/>
 *      <xsd:element name="ActivateControllerAction" type="ActivateControllerAction"/>
 *      <xsd:element name="ControllerAction" type="ControllerAction"/>
 *      <xsd:element name="TeleportAction" type="TeleportAction"/>
 *      <xsd:element name="RoutingAction" type="RoutingAction"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct PrivateAction : public ComplexType
{
  explicit PrivateAction(const pugi::xml_node &, Scope &);

  auto endsImmediately() const -> bool;

  auto run() -> void;

  auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  PrivateAction,
  CASE(LongitudinalAction),  //
  CASE(LateralAction),       //
  // CASE(VisibilityAction),
  // CASE(SynchronizeAction),
  // CASE(ActivateControllerAction),
  CASE(ControllerAction),  //
  CASE(TeleportAction),    //
  CASE(RoutingAction),     //
);

DEFINE_LAZY_VISITOR(
  const PrivateAction,
  CASE(LongitudinalAction),  //
  CASE(LateralAction),       //
  // CASE(VisibilityAction),
  // CASE(SynchronizeAction),
  // CASE(ActivateControllerAction),
  CASE(ControllerAction),  //
  CASE(TeleportAction),    //
  CASE(RoutingAction),     //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_ACTION_HPP_
