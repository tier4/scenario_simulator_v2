#ifndef SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_

#include <scenario_runner/validator/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TrafficSignalStateAction =============================================
 *
 * <xsd:complexType name="TrafficSignalStateAction">
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalStateAction
{
  const std::string name;
  const std::string state;

  template<typename Node, typename ... Ts>
  explicit TrafficSignalStateAction(const Node & node, Ts && ...)
  : name{readRequiredAttribute<std::string>(node, "name")},
    state{readRequiredAttribute<std::string>(node, "state")}
  {}

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
