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

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ---- PrivateAction ----------------------------------------------------------
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
 * -------------------------------------------------------------------------- */
struct PrivateAction
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit PrivateAction(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("LongitudinalAction", [&](auto && node) {
          return make<LongitudinalAction>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("LateralAction", [&](auto && node) {
          return make<LateralAction>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("VisibilityAction", UNSUPPORTED()),
        std::make_pair("SynchronizeAction", UNSUPPORTED()),
        std::make_pair("ActivateControllerAction", UNSUPPORTED()),
        std::make_pair("ControllerAction", UNSUPPORTED()),
        std::make_pair("TeleportAction", [&](auto && node) {
          return make<TeleportAction>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("RoutingAction", [&](auto && node) {
          return make<RoutingAction>(node, std::forward<decltype(xs)>(xs)...);
        })))
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PRIVATE_ACTION_HPP_
