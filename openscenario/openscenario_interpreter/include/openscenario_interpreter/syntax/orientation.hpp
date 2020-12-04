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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ORIENTATION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ORIENTATION_HPP_

#include <geometry_msgs/msg/vector3.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/reference_context.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Orientation ----------------------------------------------------------
 *
 *  <xsd:complexType name="Orientation">
 *    <xsd:attribute name="type" type="ReferenceContext" use="optional"/>
 *    <xsd:attribute name="h" type="Double" use="optional"/>
 *    <xsd:attribute name="p" type="Double" use="optional"/>
 *    <xsd:attribute name="r" type="Double" use="optional"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Orientation
{
  const ReferenceContext type;

  const Double h, p, r;

  Orientation() = default;

  template
  <
    typename Node,
    typename Scope
  >
  explicit Orientation(const Node & node, Scope & scope)
  : type(
      readAttribute<ReferenceContext>("type", node, scope, ReferenceContext::relative)),
    h(
      readAttribute<Double>("h", node, scope, Double())),
    p(
      readAttribute<Double>("p", node, scope, Double())),
    r(
      readAttribute<Double>("r", node, scope, Double()))
  {}

  operator geometry_msgs::msg::Vector3() const
  {
    geometry_msgs::msg::Vector3 result {};

    switch (type) {
      case ReferenceContext::relative:
        result.x = h;
        result.y = p;
        result.z = r;
        break;

      case ReferenceContext::absolute:
      // Jumps can never reach here (see ReferenceContext::operator >>).

      default:
        break;
    }

    return result;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ORIENTATION_HPP_
