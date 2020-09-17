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

#ifndef SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// #include <quaternion_operation/quaternion_operation.h>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== WorldPosition ========================================================
 *
 * <xsd:complexType name="WorldPosition">
 *   <xsd:attribute name="x" type="Double" use="required"/>
 *   <xsd:attribute name="y" type="Double" use="required"/>
 *   <xsd:attribute name="z" type="Double" use="optional"/>
 *   <xsd:attribute name="h" type="Double" use="optional"/>
 *   <xsd:attribute name="p" type="Double" use="optional"/>
 *   <xsd:attribute name="r" type="Double" use="optional"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct WorldPosition
{
  const Double x, y, z, h, p, r;

  template<typename Node, typename Scope>
  explicit WorldPosition(const Node & node, Scope & scope)
  : x{readAttribute<Double>(node, scope, "x")},
    y{readAttribute<Double>(node, scope, "y")},
    z{readAttribute<Double>(node, scope, "z", 0)},
    h{readAttribute<Double>(node, scope, "h", 0)},       // yaw
    p{readAttribute<Double>(node, scope, "p", 0)},
    r{readAttribute<Double>(node, scope, "r", 0)}
  {}

  operator geometry_msgs::msg::Pose() const
  {
    geometry_msgs::msg::Vector3 vector {};
    vector.x = r;
    vector.y = p;
    vector.z = h;

    geometry_msgs::msg::Point point {};
    point.x = x;
    point.y = y;
    point.z = z;

    geometry_msgs::msg::Pose pose {};
    pose.position = point;
    // pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(vector);

    return pose;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_
