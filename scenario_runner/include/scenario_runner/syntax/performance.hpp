#ifndef SCENARIO_RUNNER__SYNTAX__PERFORMANCE_HPP_
#define SCENARIO_RUNNER__SYNTAX__PERFORMANCE_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/reader/element.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Performance ==========================================================
 *
 * <xsd:complexType name="Performance">
 *   <xsd:attribute name="maxSpeed" type="Double" use="required"/>
 *   <xsd:attribute name="maxAcceleration" type="Double" use="required"/>
 *   <xsd:attribute name="maxDeceleration" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Performance
{
  const Double max_speed;
  const Double max_acceleration;
  const Double max_deceleration;

  Performance() = default;

  template<typename Node, typename Scope>
  explicit Performance(const Node & node, Scope & scope)
  : max_speed{readAttribute<Double>(node, scope, "maxSpeed")},
    max_acceleration{readAttribute<Double>(node, scope, "maxAcceleration")},
    max_deceleration{readAttribute<Double>(node, scope, "maxDeceleration")}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Performance & rhs)
{
  return os << indent << blue << "<Performance" << " " << highlight("maxSpeed", rhs.max_speed) <<
         " " << highlight("maxAcceleration", rhs.max_acceleration) <<
         " " << highlight("maxDeceleration", rhs.max_deceleration) << blue << "/>" << reset;
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PERFORMANCE_HPP_
