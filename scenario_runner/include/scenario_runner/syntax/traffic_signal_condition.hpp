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

#ifndef SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== TrafficSignalCondition ===============================================
 *
 * <xsd:complexType name="TrafficSignalCondition">
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalCondition
{
  const String name;

  const String state;

  template<typename Node, typename Scope>
  explicit TrafficSignalCondition(const Node & node, Scope & scope)
  : name{readAttribute<String>(node, scope, "name")},
    state{readAttribute<String>(node, scope, "state")}
  {}

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
