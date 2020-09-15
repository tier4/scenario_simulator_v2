#ifndef SCENARIO_RUNNER__SYNTAX__ENTITY_SELECTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITY_SELECTION_HPP_

#include <scenario_runner/syntax/selected_entities.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== EntitySelection ======================================================
   *
   * <xsd:complexType name="EntitySelection">
   *   <xsd:sequence>
   *     <xsd:element name="Members" type="SelectedEntities"/>
   *   </xsd:sequence>
   *   <xsd:attribute name="name" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct EntitySelection
  {};

}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_SELECTION_HPP_

