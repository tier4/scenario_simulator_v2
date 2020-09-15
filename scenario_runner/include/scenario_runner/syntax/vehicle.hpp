#ifndef SCENARIO_RUNNER__SYNTAX__VEHICLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__VEHICLE_HPP_

#include <scenario_runner/syntax/axles.hpp>
#include <scenario_runner/syntax/bounding_box.hpp>
#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/performance.hpp>
#include <scenario_runner/syntax/properties.hpp>
#include <scenario_runner/syntax/vehicle_category.hpp>

namespace scenario_runner { inline namespace syntax
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

    template <typename Node, typename Scope>
    explicit Vehicle(const Node& node, Scope& outer_scope)
      : name                   { readAttribute<String>             (node, outer_scope, "name") }
      , vehicle_category       { readAttribute<VehicleCategory>    (node, outer_scope, "vehicleCategory") }
      , inner_scope            { outer_scope }
      , parameter_declarations { readElement<ParameterDeclarations>("ParameterDeclarations", node, inner_scope) }
      , bounding_box           { readElement<BoundingBox>          ("BoundingBox",           node, inner_scope) }
      , performance            { readElement<Performance>          ("Performance",           node, inner_scope) }
      , axles                  { readElement<Axles>                ("Axles",                 node, inner_scope) }
      , properties             { readElement<Properties>           ("Properties",            node, inner_scope) }
    {}
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Vehicle& rhs)
  {
    return os << (indent++) << blue << "<Vehicle" << " " << highlight("name", rhs.name)
                                                  << " " << highlight("vehicleCategory", rhs.vehicle_category) << blue << ">\n" << reset
              << rhs.parameter_declarations << "\n"
              << rhs.bounding_box << "\n"
              << rhs.performance << "\n"
              << rhs.axles << "\n"
              // TODO properties
              << (--indent) << blue << "</Vehicle>" << reset;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__VEHICLE_HPP_
