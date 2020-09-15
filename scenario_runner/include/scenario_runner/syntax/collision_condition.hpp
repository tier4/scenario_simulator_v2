#ifndef SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_

#include <scenario_runner/syntax/entity_ref.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== CollisionCondition ===================================================
 *
 * <xsd:complexType name="CollisionCondition">
 *   <xsd:choice>
 *     <xsd:element name="EntityRef" type="EntityRef"/>
 *     <xsd:element name="ByType" type="ByObjectType"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct CollisionCondition
  : public Object
{
  EntityRef entity_ref;

  template<typename Node, typename Scope>
  explicit CollisionCondition(const Node & node, Scope & scope, const TriggeringEntities &)
  : entity_ref{readElement<EntityRef>("EntityRef", node, scope)}
  {
    callWithElements(node, "ByType", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }

  auto evaluate() const noexcept
  {
    return false_v;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_
