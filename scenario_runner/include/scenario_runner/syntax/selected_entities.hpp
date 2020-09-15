#ifndef SCENARIO_RUNNER__SYNTAX__SELECTED_ENTITIES_HPP_
#define SCENARIO_RUNNER__SYNTAX__SELECTED_ENTITIES_HPP_

#include <scenario_runner/syntax/by_type.hpp>
#include <scenario_runner/syntax/entity_ref.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== SelectedEntities =====================================================
 *
 * <xsd:complexType name="SelectedEntities">
 *   <xsd:choice>
 *     <xsd:element name="EntityRef" minOccurs="0" maxOccurs="unbounded" type="EntityRef"/>
 *     <xsd:element name="ByType" minOccurs="0" maxOccurs="unbounded" type="ByType"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct SelectedEntities
{};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SELECTED_ENTITIES_HPP_
