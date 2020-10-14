// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_

#include <open_scenario_interpreter/syntax/axles.hpp>
#include <open_scenario_interpreter/syntax/bounding_box.hpp>
#include <open_scenario_interpreter/syntax/parameter_declarations.hpp>
#include <open_scenario_interpreter/syntax/performance.hpp>
#include <open_scenario_interpreter/syntax/properties.hpp>
#include <open_scenario_interpreter/syntax/vehicle_category.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Vehicle ==============================================================
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
 * ======================================================================== */
struct Vehicle
{
  const String name;

  const VehicleCategory vehicle_category;

  Scope inner_scope;

  const ParameterDeclarations parameter_declarations;

  const BoundingBox bounding_box;

  const Performance performance;

  const Axles axles;

  const Properties properties;

  template<typename Node, typename Scope>
  explicit Vehicle(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>("name", node, outer_scope)},
    vehicle_category{readAttribute<VehicleCategory>("vehicleCategory", node, outer_scope)},
    inner_scope{outer_scope},
    parameter_declarations{
      readElement<ParameterDeclarations>("ParameterDeclarations", node, inner_scope)},
    bounding_box{readElement<BoundingBox>("BoundingBox", node, inner_scope)},
    performance{readElement<Performance>("Performance", node, inner_scope)},
    axles{readElement<Axles>("Axles", node, inner_scope)},
    properties{readElement<Properties>("Properties", node, inner_scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Vehicle & rhs)
{
  return os << (indent++) << blue << "<Vehicle" << " " << highlight("name", rhs.name) <<
         " " << highlight("vehicleCategory", rhs.vehicle_category) << blue << ">\n" << reset <<
         rhs.parameter_declarations << "\n" <<
         rhs.bounding_box << "\n" <<
         rhs.performance << "\n" <<
         rhs.axles << "\n" <<
         (--indent) << blue << "</Vehicle>" << reset;
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__VEHICLE_HPP_
