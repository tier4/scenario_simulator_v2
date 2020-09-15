#ifndef SCENARIO_RUNNER__SYNTAX__BY_ENTITY_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__BY_ENTITY_CONDITION_HPP_

#include <scenario_runner/syntax/entity_condition.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== ByEntityCondition ====================================================
 *
 * <xsd:complexType name="ByEntityCondition">
 *   <xsd:all>
 *     <xsd:element name="TriggeringEntities" type="TriggeringEntities"/>
 *     <xsd:element name="EntityCondition" type="EntityCondition"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct ByEntityCondition
{
  Scope inner_scope;

  const EntityCondition entity_condition;

  template<typename Node>
  explicit ByEntityCondition(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope},
    entity_condition{
      readElement<EntityCondition>("EntityCondition", node, inner_scope,
        readElement<TriggeringEntities>("TriggeringEntities", node, inner_scope))
    }
  {}

  template<typename ... Ts>
  auto evaluate(Ts && ... xs) const
  {
    return entity_condition.evaluate(std::forward<decltype(xs)>(xs)...);
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__BY_ENTITY_CONDITION_HPP_
