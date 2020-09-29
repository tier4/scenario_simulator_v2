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

#ifndef SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_

#include <scenario_runner/syntax/acquire_position_action.hpp>
#include <scenario_runner/syntax/assign_route_action.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== RoutingAction ========================================================
 *
 * <xsd:complexType name="RoutingAction">
 *   <xsd:choice>
 *     <xsd:element name="AssignRouteAction" type="AssignRouteAction"/>
 *     <xsd:element name="FollowTrajectoryAction" type="FollowTrajectoryAction"/>
 *     <xsd:element name="AcquirePositionAction" type="AcquirePositionAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct RoutingAction
  : public Element
{
  template<typename Node, typename Scope>
  explicit RoutingAction(const Node & node, Scope & outer_scope)
  {
    callWithElements(
      node, "AssignRouteAction", 0, 1, [&](auto && node)
      {
        return rebind<AssignRouteAction>(node, outer_scope);
      });

    callWithElements(
      node, "FollowTrajectoryAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(
      node, "AcquirePositionAction", 0, 1, [&](auto && node)
      {
        return rebind<AcquirePositionAction>(node, outer_scope);
      });
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_
