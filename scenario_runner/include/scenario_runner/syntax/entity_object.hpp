#ifndef SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_

#include <scenario_runner/syntax/pedestrian.hpp>
#include <scenario_runner/syntax/vehicle.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== EntityObject =========================================================
 *
 * <xsd:group name="EntityObject">
 *   <xsd:choice>
 *     <xsd:element name="CatalogReference" type="CatalogReference"/>
 *     <xsd:element name="Vehicle" type="Vehicle"/>
 *     <xsd:element name="Pedestrian" type="Pedestrian"/>
 *     <xsd:element name="MiscObject" type="MiscObject"/>
 *   </xsd:choice>
 * </xsd:group>
 *
 * ======================================================================== */
struct EntityObject
  : public Object
{
  template<typename Node>
  explicit EntityObject(const Node & node, Scope & scope)
  {
    callWithElements(node, "CatalogReference", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "Vehicle", 0, 1, [&](auto && element)
      {
        return rebind<Vehicle>(element, scope);
      });

    callWithElements(node, "Pedestrian", 0, 1, [&](auto && element) mutable
      {
        return rebind<Pedestrian>(element, scope);
      });

    callWithElements(node, "MiscObject", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_
