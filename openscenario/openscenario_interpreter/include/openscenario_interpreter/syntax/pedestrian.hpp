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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/pedestrian_category.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <pugixml.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Pedestrian -------------------------------------------------------------
 *
 *  <xsd:complexType name="Pedestrian">
 *    <xsd:all>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="BoundingBox" type="BoundingBox"/>
 *      <xsd:element name="Properties" type="Properties"/>
 *    </xsd:all>
 *    <xsd:attribute name="model" type="String" use="required"/>
 *    <xsd:attribute name="model3d" type="String" />
 *    <xsd:attribute name="mass" type="Double" use="required"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="pedestrianCategory" type="PedestrianCategory" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Pedestrian : public Scope
{
  static constexpr ObjectType object_type{ObjectType::pedestrian};

  const Double mass;

  const String model;

  const String model3d;

  const PedestrianCategory pedestrian_category;

  const ParameterDeclarations parameter_declarations;

  const BoundingBox bounding_box;

  const Properties properties;

  explicit Pedestrian(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator_msgs::msg::PedestrianParameters() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_
