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

#ifndef SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_
#define SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_

#include <scenario_runner/syntax/bounding_box.hpp>
#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/pedestrian_category.hpp>
#include <scenario_runner/syntax/properties.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== Pedestrian ===========================================================
 *
 * <xsd:complexType name="Pedestrian">
 *   <xsd:all>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="BoundingBox" type="BoundingBox"/>
 *     <xsd:element name="Properties" type="Properties"/>
 *   </xsd:all>
 *   <xsd:attribute name="model" type="String" use="required"/>
 *   <xsd:attribute name="mass" type="Double" use="required"/>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="pedestrianCategory" type="PedestrianCategory" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Pedestrian
{
  const String name;

  const Double mass;

  const String model;

  const PedestrianCategory pedestrian_category;

  Scope inner_scope;

  const ParameterDeclarations parameter_declarations;

  const BoundingBox bounding_box;

  const Properties properties;

  template<typename Node, typename Scope>
  explicit Pedestrian(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>(node, outer_scope, "name")},
    mass{readAttribute<Double>(node, outer_scope, "mass")},
    model{readAttribute<String>(node, outer_scope, "model")},
    pedestrian_category{readAttribute<PedestrianCategory>(node, outer_scope, "pedestrianCategory")},
    inner_scope{outer_scope},
    parameter_declarations{readElement<ParameterDeclarations>("ParameterDeclarations", node,
        inner_scope)},
    bounding_box{readElement<BoundingBox>("BoundingBox", node, inner_scope)},
    properties{readElement<Properties>("Properties", node, inner_scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Pedestrian & rhs)
{
  return os << (indent++) << blue << "<Pedestrian" << " " << highlight("name", rhs.name) <<
         " " << highlight("mass", rhs.mass) <<
         " " << highlight("model", rhs.model) <<
         " " <<
         highlight("pedestrianCategory", rhs.pedestrian_category) << blue << ">\n" << reset <<
         rhs.parameter_declarations << "\n" <<
         rhs.bounding_box << "\n" <<
         (--indent) << blue << "</Pedestrian>" << reset;
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_
