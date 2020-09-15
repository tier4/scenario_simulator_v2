#ifndef SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
#define SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== AbsoluteTargetLane ===================================================
 *
 * <xsd:complexType name="AbsoluteTargetLane">
 *   <xsd:attribute name="value" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct AbsoluteTargetLane
{
  const String value;

  template<typename Node, typename Scope>
  explicit AbsoluteTargetLane(const Node & node, Scope & scope)
  : value{readAttribute<std::decay<decltype(value)>::type>(node, scope, "value")}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
