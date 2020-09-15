#ifndef SCENARIO_RUNNER__SYNTAX__WAYPOINT_HPP_
#define SCENARIO_RUNNER__SYNTAX__WAYPOINT_HPP_

#include <scenario_runner/syntax/position.hpp>
#include <scenario_runner/syntax/route_strategy.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Waypoint =============================================================
 *
 * <xsd:complexType name="Waypoint">
 *   <xsd:sequence>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="routeStrategy" type="RouteStrategy" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Waypoint
  : public Sequence
{
  const RouteStrategy route_strategy;

  template<typename Node, typename ... Ts>
  explicit Waypoint(const Node & node, Ts && ... xs)
  : route_strategy{readRequiredAttribute<std::decay<decltype(route_strategy)>::type>(node,
        "routeStrategy")}
  {
    defineElement<Position>("Position");

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__WAYPOINT_HPP_
