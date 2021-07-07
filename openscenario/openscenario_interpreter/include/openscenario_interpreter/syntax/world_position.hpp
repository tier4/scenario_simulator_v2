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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__WORLD_POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__WORLD_POSITION_HPP_

#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <openscenario_msgs/msg/lanelet_pose.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- WorldPosition ----------------------------------------------------------
 *
 *  <xsd:complexType name="WorldPosition">
 *    <xsd:attribute name="x" type="Double" use="required"/>
 *    <xsd:attribute name="y" type="Double" use="required"/>
 *    <xsd:attribute name="z" type="Double" use="optional"/>
 *    <xsd:attribute name="h" type="Double" use="optional"/>
 *    <xsd:attribute name="p" type="Double" use="optional"/>
 *    <xsd:attribute name="r" type="Double" use="optional"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct WorldPosition
{
  const Double x, y, z, h, p, r;

  template <typename Node, typename Scope>
  explicit WorldPosition(const Node & node, Scope & scope)
  : x(readAttribute<Double>("x", node, scope)),
    y(readAttribute<Double>("y", node, scope)),
    z(readAttribute<Double>("z", node, scope, Double())),
    h(readAttribute<Double>("h", node, scope, Double())),  // yaw
    p(readAttribute<Double>("p", node, scope, Double())),
    r(readAttribute<Double>("r", node, scope, Double()))
  {
  }

  explicit operator geometry_msgs::msg::Pose() const
  {
    geometry_msgs::msg::Vector3 vector;
    {
      vector.x = r;
      vector.y = p;
      vector.z = h;
    }

    geometry_msgs::msg::Point point;
    {
      point.x = x;
      point.y = y;
      point.z = z;
    }

    geometry_msgs::msg::Pose pose;
    {
      pose.position = point;
      pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(vector);
    }

    return pose;
  }

  explicit operator openscenario_msgs::msg::LaneletPose() const
  {
    return toLanePosition(static_cast<geometry_msgs::msg::Pose>(*this));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WORLD_POSITION_HPP_
