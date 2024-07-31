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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/misc_object_category.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- MiscObject 1.1 ---------------------------------------------------------
 *
 *  <xsd:complexType name="MiscObject">
 *    <xsd:all>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="BoundingBox" type="BoundingBox"/>
 *      <xsd:element name="Properties" type="Properties"/>
 *    </xsd:all>
 *    <xsd:attribute name="mass" type="Double" use="required"/>
 *    <xsd:attribute name="miscObjectCategory" type="MiscObjectCategory" use="required"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="model3d" type="String"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct MiscObject : public Scope
{
  static constexpr ObjectType object_type{ObjectType::miscellaneous};

  const Double mass;  // Mass of the miscellaneous object. Unit: Kg; Range: [0..inf[.

  const MiscObjectCategory misc_object_category;  // Categorization of the miscellaneous object.

  const String
    model3d;  // Definition of the model of the miscellaneous object as a model type or a relative or absolute file path.

  const ParameterDeclarations parameter_declarations;  // Definition of additional parameters.

  const BoundingBox bounding_box;  // Bounding box definition for the miscellaneous object.

  Properties properties;  // Property definitions for the miscellaneous object.

  explicit MiscObject(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator_msgs::msg::MiscObjectParameters() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_HPP_
