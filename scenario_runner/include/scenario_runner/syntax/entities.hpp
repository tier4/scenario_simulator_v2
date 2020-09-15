#ifndef SCENARIO_RUNNER__SYNTAX__ENTITIES_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITIES_HPP_

#include <scenario_runner/syntax/entity_selection.hpp>
#include <scenario_runner/syntax/scenario_object.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Entities =============================================================
 *
 * <xsd:complexType name="Entities">
 *   <xsd:sequence>
 *     <xsd:element name="ScenarioObject" minOccurs="0" maxOccurs="unbounded" type="ScenarioObject"/>
 *     <xsd:element name="EntitySelection" minOccurs="0" maxOccurs="unbounded" type="EntitySelection"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Entities
{
  template<typename Node>
  explicit Entities(const Node & parent, Scope & scope)
  {
    callWithElements(parent, "ScenarioObject", 0, unbounded, [&](auto && child)
      {
        scope.entities.emplace(readAttribute<String>(child, scope, "name"),
        make<ScenarioObject>(child, scope));
      });

    callWithElements(parent, "EntitySelection", 0, unbounded, THROW_UNSUPPORTED_ERROR(parent));
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITIES_HPP_
