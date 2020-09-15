#ifndef SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_

#include <scenario_runner/syntax/traffic_signal_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== InfrastructureAction =================================================
 *
 * <xsd:complexType name="InfrastructureAction">
 *   <xsd:all>
 *     <xsd:element name="TrafficSignalAction" type="TrafficSignalAction"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct InfrastructureAction
  : public All
{
  template<typename Node, typename ... Ts>
  explicit InfrastructureAction(const Node & node, Ts && ... xs)
  {
    defineElement<TrafficSignalAction>("TrafficSignalAction");

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
