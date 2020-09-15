#ifndef SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TrafficSignalCondition ===============================================
 *
 * <xsd:complexType name="TrafficSignalCondition">
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalCondition
{
  const String name;

  const String state;

  template<typename Node, typename Scope>
  explicit TrafficSignalCondition(const Node & node, Scope & scope)
  : name{readAttribute<String>(node, scope, "name")},
    state{readAttribute<String>(node, scope, "state")}
  {}

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};

}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
