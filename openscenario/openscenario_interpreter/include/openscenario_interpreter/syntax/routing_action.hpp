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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_

#include <openscenario_interpreter/syntax/acquire_position_action.hpp>
#include <openscenario_interpreter/syntax/assign_route_action.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RoutingAction ----------------------------------------------------------
 *
 * <xsd:complexType name="RoutingAction">
 *   <xsd:choice>
 *     <xsd:element name="AssignRouteAction" type="AssignRouteAction"/>
 *     <xsd:element name="FollowTrajectoryAction" type="FollowTrajectoryAction"/>
 *     <xsd:element name="AcquirePositionAction" type="AcquirePositionAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RoutingAction : public ComplexType
{
  template <typename Node, typename... Ts>
  explicit RoutingAction(const Node & node, Ts &&... xs)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair(     "AssignRouteAction", [&](auto && node) { return make<     AssignRouteAction>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("FollowTrajectoryAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair( "AcquirePositionAction", [&](auto && node) { return make< AcquirePositionAction>(node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
  }

  auto endsImmediately() const -> bool;

  auto run() -> void;

  auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  RoutingAction,
  CASE(AssignRouteAction),  //
  // CASE(FollowTrajectoryAction),
  CASE(AcquirePositionAction),  //
);

DEFINE_LAZY_VISITOR(
  const RoutingAction,
  CASE(AssignRouteAction),  //
  // CASE(FollowTrajectoryAction),
  CASE(AcquirePositionAction),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_
