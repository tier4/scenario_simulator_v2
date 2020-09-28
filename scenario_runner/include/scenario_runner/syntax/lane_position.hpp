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

#ifndef SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_

#include <scenario_runner/syntax/orientation.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== LanePosition =========================================================
 *
 * <xsd:complexType name="LanePosition">
 *   <xsd:all>
 *     <xsd:element name="Orientation" type="Orientation" minOccurs="0"/>
 *   </xsd:all>
 *   <xsd:attribute name="roadId" type="String" use="required"/>
 *   <xsd:attribute name="laneId" type="String" use="required"/>
 *   <xsd:attribute name="offset" type="Double" use="optional"/>
 *   <xsd:attribute name="s" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct LanePosition
{
  const String road_id, lane_id;

  const Double offset, s;

  const Orientation orientation;

  template<typename Node, typename Scope>
  explicit LanePosition(const Node & node, Scope & scope)
  : road_id{readAttribute<String>(node, scope, "roadId", "")},
    lane_id{readAttribute<String>(node, scope, "laneId")},
    offset{readAttribute<Double>(node, scope, "offset", Double())},
    s{readAttribute<Double>(node, scope, "s")},
    orientation{readElement<Orientation>("Orientation", node, scope)}
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_
