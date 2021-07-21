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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_WORLD_POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_WORLD_POSITION_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_msgs/msg/lanelet_pose.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeWorldPosition --------------------------------------------------
 *
 *  <xsd:complexType name="RelativeWorldPosition">
 *    <xsd:all>
 *      <xsd:element name="Orientation" type="Orientation" minOccurs="0"/>
 *    </xsd:all>
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="dx" type="Double" use="required"/>
 *    <xsd:attribute name="dy" type="Double" use="required"/>
 *    <xsd:attribute name="dz" type="Double" use="optional"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeWorldPosition
{
  const Orientation orientation;

  const EntityRef reference;

  const Double dx, dy, dz;

  template <typename Node, typename Scope>
  explicit RelativeWorldPosition(const Node & node, Scope & scope)
  : orientation(readElement<Orientation>("Orientation", node, scope)),
    reference(readAttribute<EntityRef>("entityRef", node, scope)),
    dx(readAttribute<Double>("dx", node, scope)),
    dy(readAttribute<Double>("dy", node, scope)),
    dz(readAttribute<Double>("dz", node, scope, Double()))
  {
  }

  operator geometry_msgs::msg::Point() const
  {
    geometry_msgs::msg::Point result;
    {
      result.x = dx;
      result.y = dy;
      result.z = dz;
    }

    return result;
  }

  explicit operator geometry_msgs::msg::Pose() const
  {
    throw UNSUPPORTED_CONVERSION_DETECTED(RelativeWorldPosition, geometry_msgs::msg::Pose);
  }

  explicit operator openscenario_msgs::msg::LaneletPose() const
  {
    throw UNSUPPORTED_CONVERSION_DETECTED(
      RelativeWorldPosition, openscenario_msgs::msg::LaneletPose);
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_WORLD_POSITION_HPP_
