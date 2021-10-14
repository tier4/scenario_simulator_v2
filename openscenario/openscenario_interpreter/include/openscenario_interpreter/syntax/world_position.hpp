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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_msgs/msg/lanelet_pose.hpp>
#include <pugixml.hpp>

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

  explicit WorldPosition(const pugi::xml_node &, Scope &);

  explicit operator geometry_msgs::msg::Pose() const;

  explicit operator openscenario_msgs::msg::LaneletPose() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WORLD_POSITION_HPP_
