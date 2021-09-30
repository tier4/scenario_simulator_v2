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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_

#include <geometry_msgs/msg/vector3.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/double.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Dimensions -------------------------------------------------------------
 *
 *  <xsd:complexType name="Dimensions">
 *    <xsd:attribute name="width"  type="Double" use="required"/>
 *    <xsd:attribute name="length" type="Double" use="required"/>
 *    <xsd:attribute name="height" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * ------------------------------------------------------------------------ */
struct Dimensions
{
  const Double width, length, height;

  Dimensions() = default;

  template <typename Node, typename Scope>
  explicit Dimensions(const Node & node, Scope & scope)
  : width(readAttribute<Double>("width", node, scope)),
    length(readAttribute<Double>("length", node, scope)),
    height(readAttribute<Double>("height", node, scope))
  {
  }

  explicit operator geometry_msgs::msg::Vector3() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_
