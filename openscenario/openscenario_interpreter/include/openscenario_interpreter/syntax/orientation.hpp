// Copyright 2015 TIER IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/reference_context.hpp>
#include <pugixml.hpp>

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
  const ReferenceContext type{};

  Double h, p, r;

  Orientation() = default;

  explicit Orientation(const pugi::xml_node &, Scope &);

  operator geometry_msgs::msg::Vector3() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ORIENTATION_HPP_
