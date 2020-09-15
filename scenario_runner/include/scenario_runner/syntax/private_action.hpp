// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__PRIVATE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__PRIVATE_ACTION_HPP_

#include <scenario_runner/syntax/lateral_action.hpp>
#include <scenario_runner/syntax/longitudinal_action.hpp>
#include <scenario_runner/syntax/routing_action.hpp>
#include <scenario_runner/syntax/teleport_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== PrivateAction ========================================================
 *
 * <xsd:complexType name="PrivateAction">
 *   <xsd:choice>
 *     <xsd:element name="LongitudinalAction" type="LongitudinalAction"/>
 *     <xsd:element name="LateralAction" type="LateralAction"/>
 *     <xsd:element name="VisibilityAction" type="VisibilityAction"/>
 *     <xsd:element name="SynchronizeAction" type="SynchronizeAction"/>
 *     <xsd:element name="ActivateControllerAction" type="ActivateControllerAction"/>
 *     <xsd:element name="ControllerAction" type="ControllerAction"/>
 *     <xsd:element name="TeleportAction" type="TeleportAction"/>
 *     <xsd:element name="RoutingAction" type="RoutingAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct PrivateAction
  : public Object
{
  template<typename Node, typename Scope>
  explicit PrivateAction(const Node & node, Scope & scope)
  {
    callWithElements(node, "LongitudinalAction", 0, 1, [&](auto && node)
      {
        return rebind<LongitudinalAction>(node, scope);
      });

    callWithElements(node, "LateralAction", 0, 1, [&](auto && node)
      {
        return rebind<LateralAction>(node, scope);
      });

    callWithElements(node, "VisibilityAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "SynchronizeAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "ActivateControllerAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "ControllerAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "TeleportAction", 0, 1, [&](auto && node)
      {
        return rebind<TeleportAction>(node, scope);
      });

    callWithElements(node, "RoutingAction", 0, 1, [&](auto && node)
      {
        return rebind<RoutingAction>(node, scope);
      });
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PRIVATE_ACTION_HPP_
