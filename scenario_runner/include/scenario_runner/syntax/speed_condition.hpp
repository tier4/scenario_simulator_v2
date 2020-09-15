#ifndef SCENARIO_RUNNER__SYNTAX__SPEED_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__SPEED_CONDITION_HPP_

#include <scenario_runner/syntax/rule.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== SpeedCondition =======================================================
 *
 * <xsd:complexType name="SpeedCondition">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct SpeedCondition
{
  const Double value;

  const Rule compare;

  Scope inner_scope;

  const TriggeringEntities trigger;

  template<typename Node>
  explicit SpeedCondition(
    const Node & node, Scope & outer_scope,
    const TriggeringEntities & trigger)
  : value{readAttribute<Double>(node, outer_scope, "value")},
    compare{readAttribute<Rule>(node, outer_scope, "rule")},
    inner_scope{outer_scope},
    trigger{trigger}
  {}

  auto evaluate()
  {
    // return
    //   asBoolean(
    //     trigger([&](auto&& entity)
    //     {
    //       return compare(inner_scope.getEntityStatus(entity).twist.linear.x, value);
    //     }));

    return false_v;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SPEED_CONDITION_HPP_
