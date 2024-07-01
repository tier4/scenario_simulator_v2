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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/axles.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/performance.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/vehicle_category.hpp>
#include <pugixml.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

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
 *   <xsd:attribute name="model3d" type="String"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Vehicle : public Scope  // for ParameterDeclarations
{
  static constexpr ObjectType object_type{ObjectType::vehicle};

  const VehicleCategory vehicle_category;  // Category of the vehicle (bicycle, train,...).

  const String
    model3d;  // Definition of the model of the vehicle as a model type or a relative or absolute file path.

  const ParameterDeclarations parameter_declarations;  // Definition of additional parameters.

  const BoundingBox bounding_box;  // The three dimensional bounding box that encloses the vehicle.

  const Performance performance;  // Performance properties of the vehicle.

  const Axles axles;  // A set of axles (front, rear, additional) and their geometric locations.

  Properties properties;  // Additional properties as name value pairs.

  explicit Vehicle(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator_msgs::msg::VehicleParameters() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
