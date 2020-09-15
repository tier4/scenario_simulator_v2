#ifndef SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_

#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/waypoint.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Route ================================================================
 *
 * <xsd:complexType name="Route">
 *   <xsd:sequence>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="Waypoint" minOccurs="2" maxOccurs="unbounded" type="Waypoint"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="closed" type="Boolean" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Route
  : public Sequence
{
  const std::string name;

  const Boolean closed;

  template<typename Node, typename ... Ts>
  explicit Route(const Node & node, Ts && ... xs)
  : name{readRequiredAttribute<std::decay<decltype(name)>::type>(node, "name")},
    closed{readUnsupportedAttribute<std::decay<decltype(closed)>::type>(node, "closed", false)}
  {
    defineElement<ParameterDeclarations>("ParameterDeclarations", 0, 1);
    defineElement<Waypoint>("Waypoint", 2, unbounded);

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_
