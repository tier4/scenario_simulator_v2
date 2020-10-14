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

#ifndef SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_

#include <scenario_runner/syntax/traffic_signal_action.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== InfrastructureAction =================================================
 *
 * <xsd:complexType name="InfrastructureAction">
 *   <xsd:all>
 *     <xsd:element name="TrafficSignalAction" type="TrafficSignalAction"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct InfrastructureAction
  : public Element
{
  template<typename Node, typename Scope>
  explicit InfrastructureAction(const Node & node, Scope & outer_scope)
  {
    callWithElements(
      node, "TrafficSignalAction", 1, 1,
      [&](auto)
      {
        return rebind<TrafficSignalAction>(node, outer_scope);
      });
  }
};
}  // inline namespace syntax
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
