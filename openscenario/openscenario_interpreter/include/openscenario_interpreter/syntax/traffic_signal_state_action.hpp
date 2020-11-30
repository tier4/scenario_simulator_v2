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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== TrafficSignalStateAction =============================================
 *
 * <xsd:complexType name="TrafficSignalStateAction">
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalStateAction
{
  const String name;

  const String state;

  template<typename Node, typename Scope>
  explicit TrafficSignalStateAction(const Node & node, Scope & scope)
  : name{readAttribute<String>("name", node, scope)},
    state{readAttribute<String>("state", node, scope)}
  {}
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
