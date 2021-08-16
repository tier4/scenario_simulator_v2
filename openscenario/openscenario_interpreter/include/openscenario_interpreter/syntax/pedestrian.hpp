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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_

#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/pedestrian_category.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_msgs/msg/pedestrian_parameters.hpp>

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
 *    <xsd:attribute name="mass" type="Double" use="required"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="pedestrianCategory" type="PedestrianCategory" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Pedestrian : public Scope
{
  const Double mass;

  const String model;

  const PedestrianCategory pedestrian_category;

  const ParameterDeclarations parameter_declarations;

  const BoundingBox bounding_box;

  const Properties properties;

  template <typename Node>
  explicit Pedestrian(const Node & node, Scope & outer_scope)
  : Scope(outer_scope.makeChildScope(readAttribute<String>("name", node, outer_scope))),
    mass(readAttribute<Double>("mass", node, localScope())),
    model(readAttribute<String>("model", node, localScope())),
    pedestrian_category(
      readAttribute<PedestrianCategory>("pedestrianCategory", node, localScope())),
    parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, localScope())),
    bounding_box(readElement<BoundingBox>("BoundingBox", node, localScope())),
    properties(readElement<Properties>("Properties", node, localScope()))
  {
  }

  explicit operator openscenario_msgs::msg::PedestrianParameters() const
  {
    openscenario_msgs::msg::PedestrianParameters parameter;
    {
      parameter.name = name;
      parameter.pedestrian_category = boost::lexical_cast<String>(pedestrian_category);
      parameter.bounding_box = static_cast<openscenario_msgs::msg::BoundingBox>(bounding_box);
    }

    return parameter;
  }
};

std::ostream & operator<<(std::ostream &, const Pedestrian &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_HPP_
