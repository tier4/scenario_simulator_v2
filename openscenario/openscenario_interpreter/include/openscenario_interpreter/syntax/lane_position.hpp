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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/orientation.hpp>
#include <openscenario_msgs/msg/lanelet_pose.hpp>
#include <traffic_simulator/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LanePosition -----------------------------------------------------------
 *
 *  <xsd:complexType name="LanePosition">
 *    <xsd:all>
 *      <xsd:element name="Orientation" type="Orientation" minOccurs="0"/>
 *    </xsd:all>
 *    <xsd:attribute name="roadId" type="String" use="required"/>
 *    <xsd:attribute name="laneId" type="String" use="required"/>
 *    <xsd:attribute name="offset" type="Double" use="optional"/>
 *    <xsd:attribute name="s" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LanePosition
{
  const String road_id, lane_id;

  const Double offset, s;

  const Orientation orientation;

  template <typename Node, typename Scope>
  explicit LanePosition(const Node & node, Scope & scope)
  : road_id(readAttribute<String>("roadId", node, scope, "none")),
    lane_id(readAttribute<String>("laneId", node, scope)),
    offset(readAttribute<Double>("offset", node, scope, Double())),
    s(readAttribute<Double>("s", node, scope)),
    orientation(readElement<Orientation>("Orientation", node, scope))
  {
  }

  explicit operator openscenario_msgs::msg::LaneletPose() const
  {
    const geometry_msgs::msg::Vector3 rpy = orientation;
    return traffic_simulator::helper::constructLaneletPose(
      static_cast<Integer>(lane_id), s, offset, rpy.x, rpy.y, rpy.z);
  }

  explicit operator geometry_msgs::msg::Pose() const
  {
    return toWorldPosition(static_cast<openscenario_msgs::msg::LaneletPose>(*this));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_
