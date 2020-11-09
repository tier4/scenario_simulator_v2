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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_

#include <openscenario_interpreter/syntax/traffic_signal_action.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- InfrastructureAction ---------------------------------------------------
 *
 * <xsd:complexType name="InfrastructureAction">
 *   <xsd:all>
 *     <xsd:element name="TrafficSignalAction" type="TrafficSignalAction"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct InfrastructureAction
  : public Element
{
  template<typename Node, typename Scope>
  explicit InfrastructureAction(const Node & node, Scope & outer_scope)
  : Element(
      readElement<TrafficSignalAction>("TrafficSignalAction", node, outer_scope))
  {}
};
}  // inline namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
