#ifndef SCENARIO_RUNNER__SYNTAX__ACTORS_HPP_
#define SCENARIO_RUNNER__SYNTAX__ACTORS_HPP_

#include <scenario_runner/syntax/entity_ref.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Actors ===============================================================
 *
 * <xsd:complexType name="Actors">
 *   <xsd:sequence>
 *     <xsd:element name="EntityRef" minOccurs="0" maxOccurs="unbounded" type="EntityRef"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="selectTriggeringEntities" type="Boolean" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Actors
{
  // Indicates whether the triggering entities are considered actors.
  const Boolean select_triggering_entities;   // TODO BOOLEAN

  template<typename Node, typename Scope>
  explicit Actors(const Node & node, Scope & scope)
  : select_triggering_entities{readAttribute<Boolean>(node, scope, "selectTriggeringEntities",
        false)}
  {
    callWithElements(node, "EntityRef", 0, unbounded, [&](auto && node)
      {
        scope.actors.emplace_back(node, scope);
      });
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ACTORS_HPP_
