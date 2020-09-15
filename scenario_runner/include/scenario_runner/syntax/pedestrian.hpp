#ifndef SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_
#define SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_

#include <scenario_runner/syntax/bounding_box.hpp>
#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/pedestrian_category.hpp>
#include <scenario_runner/syntax/properties.hpp>

namespace scenario_runner
{inline namespace syntax
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
         rhs.bounding_box << "\n"
         // TODO properties
            << (--indent) << blue << "</Pedestrian>" << reset;
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_HPP_
