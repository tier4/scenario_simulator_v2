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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__AXLE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__AXLE_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_msgs/msg/axle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Axle -------------------------------------------------------------------
 *
 *  <xsd:complexType name="Axle">
 *    <xsd:attribute name="maxSteering" type="Double" use="required"/>
 *    <xsd:attribute name="wheelDiameter" type="Double" use="required"/>
 *    <xsd:attribute name="trackWidth" type="Double" use="required"/>
 *    <xsd:attribute name="positionX" type="Double" use="required"/>
 *    <xsd:attribute name="positionZ" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Axle
{
  const Double max_steering, wheel_diameter, track_width, position_x, position_z;

  Axle() = default;

  template <typename Node, typename Scope>
  explicit Axle(const Node & node, Scope & scope)
  // clang-format off
  : max_steering  (readAttribute<Double>("maxSteering",   node, scope)),
    wheel_diameter(readAttribute<Double>("wheelDiameter", node, scope)),
    track_width   (readAttribute<Double>("trackWidth",    node, scope)),
    position_x    (readAttribute<Double>("positionX",     node, scope)),
    position_z    (readAttribute<Double>("positionZ",     node, scope))
  // clang-format on
  {
  }

  explicit operator openscenario_msgs::msg::Axle() const;
};

struct FrontAxle : public Axle
{
  using Axle::Axle;
};

struct RearAxle : public Axle
{
  using Axle::Axle;
};

struct AdditionalAxle : public Axle
{
  using Axle::Axle;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__AXLE_HPP_
