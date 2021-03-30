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
struct Vehicle
{
  /* ---- name -----------------------------------------------------------------
   *
   *  Name of the vehicle type.
   *
   * ------------------------------------------------------------------------ */
  const String name;

  /* ---- vehicleCategory ------------------------------------------------------
   *
   *  Category of the vehicle (bicycle, train,...).
   *
   * ------------------------------------------------------------------------ */
  const VehicleCategory vehicle_category;

  Scope inner_scope;

  /* ---- ParameterDeclarations ------------------------------------------------
   *
   *  Definition of additional parameters.
   *
   * ------------------------------------------------------------------------ */
  const ParameterDeclarations parameter_declarations;

  /* ---- BoundingBox ----------------------------------------------------------
   *
   *  The three dimensional bounding box that encloses the vehicle.
   *
   * ------------------------------------------------------------------------ */
  const BoundingBox bounding_box;

  /* ---- Performance ----------------------------------------------------------
   *
   *  Performance properties of the vehicle.
   *
   * ------------------------------------------------------------------------ */
  const Performance performance;

  /* ---- Axles ----------------------------------------------------------------
   *
   *  A set of axles (front, rear, additional) and their geometric locations.
   *
   * ------------------------------------------------------------------------ */
  const Axles axles;

  /* ---- Properties -----------------------------------------------------------
   *
   *  Additional properties as name value pairs.
   *
   * ------------------------------------------------------------------------ */
  Properties properties;

  template<typename Node, typename Scope>
  explicit Vehicle(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    vehicle_category(readAttribute<VehicleCategory>("vehicleCategory", node, outer_scope)),
    inner_scope(outer_scope),
    parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, inner_scope)),
    bounding_box(readElement<BoundingBox>("BoundingBox", node, inner_scope)),
    performance(readElement<Performance>("Performance", node, inner_scope)),
    axles(readElement<Axles>("Axles", node, inner_scope)),
    properties(readElement<Properties>("Properties", node, inner_scope))
  {}

  explicit operator openscenario_msgs::msg::VehicleParameters() const
  {
    openscenario_msgs::msg::VehicleParameters parameter {};
    {
      parameter.name = name;
      parameter.vehicle_category = boost::lexical_cast<String>(vehicle_category);
    }

    return parameter;
  }

  template<typename ... Ts>
  decltype(auto) operator[](Ts && ... xs)
  {
    return properties.operator[](std::forward<decltype(xs)>(xs)...);
  }
};

std::ostream & operator<<(std::ostream & os, const Vehicle & datum)
{
  os << (indent++);
  os << blue << "<Vehicle ";
  os << highlight("name", datum.name);
  os << " " << highlight("vehicleCategory", datum.vehicle_category);
  os << blue << ">\n" << reset;
  os << datum.parameter_declarations << "\n";
  os << datum.bounding_box << "\n";
  os << datum.performance << "\n";
  os << datum.axles << "\n";
  os << (--indent) << blue << "</Vehicle>" << reset;

  return os;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
