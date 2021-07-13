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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_

#include <openscenario_interpreter/syntax/axles.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/performance.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/vehicle_category.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Vehicle ----------------------------------------------------------------
 *
 * <xsd:complexType name="Vehicle">
 *   <xsd:all>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="BoundingBox" type="BoundingBox"/>
 *     <xsd:element name="Performance" type="Performance"/>
 *     <xsd:element name="Axles" type="Axles"/>
 *     <xsd:element name="Properties" type="Properties"/>
 *   </xsd:all>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="vehicleCategory" type="VehicleCategory" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Vehicle : private Scope  // for ParameterDeclarations
{
  const String name;  // Name of the vehicle type.

  const VehicleCategory vehicle_category;  // Category of the vehicle (bicycle, train,...).

  const ParameterDeclarations parameter_declarations;  // Definition of additional parameters.

  const BoundingBox bounding_box;  // The three dimensional bounding box that encloses the vehicle.

  const Performance performance;  // Performance properties of the vehicle.

  const Axles axles;  // A set of axles (front, rear, additional) and their geometric locations.

  Properties properties;  // Additional properties as name value pairs.

  template <typename Node, typename Scope>
  explicit Vehicle(const Node & node, Scope & outer_scope)
  : Scope(outer_scope),
    name(readAttribute<String>("name", node, localScope())),
    vehicle_category(readAttribute<VehicleCategory>("vehicleCategory", node, localScope())),
    parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, localScope())),
    bounding_box(readElement<BoundingBox>("BoundingBox", node, localScope())),
    performance(readElement<Performance>("Performance", node, localScope())),
    axles(readElement<Axles>("Axles", node, localScope())),
    properties(readElement<Properties>("Properties", node, localScope()))
  {
  }

  explicit operator openscenario_msgs::msg::VehicleParameters() const
  {
    openscenario_msgs::msg::VehicleParameters parameter;
    {
      parameter.name = name;
      parameter.vehicle_category = boost::lexical_cast<String>(vehicle_category);
      parameter.bounding_box = static_cast<openscenario_msgs::msg::BoundingBox>(bounding_box);
      parameter.performance = static_cast<openscenario_msgs::msg::Performance>(performance);
      parameter.axles = static_cast<openscenario_msgs::msg::Axles>(axles);
    }

    return parameter;
  }

  template <typename... Ts>
  decltype(auto) operator[](Ts &&... xs)
  {
    return properties.operator[](std::forward<decltype(xs)>(xs)...);
  }
};

std::ostream & operator<<(std::ostream &, const Vehicle &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
