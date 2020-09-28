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

#ifndef SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
#define SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== AbsoluteTargetLane ===================================================
 *
 * <xsd:complexType name="AbsoluteTargetLane">
 *   <xsd:attribute name="value" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct AbsoluteTargetLane
{
  const String value;

  template<typename Node, typename Scope>
  explicit AbsoluteTargetLane(const Node & node, Scope & scope)
  : value{readAttribute<std::decay<decltype(value)>::type>(node, scope, "value")}
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
