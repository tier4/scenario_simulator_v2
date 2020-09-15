#ifndef SCENARIO_RUNNER__SYNTAX__PROPERTIES_HPP_
#define SCENARIO_RUNNER__SYNTAX__PROPERTIES_HPP_

namespace scenario_runner
{inline namespace syntax
{
/* ==== Properties ===========================================================
 *
 * <xsd:complexType name="Properties">
 *   <xsd:sequence>
 *     <xsd:element name="Property" minOccurs="0" maxOccurs="unbounded" type="Property"/>
 *     <xsd:element name="File" type="File" minOccurs="0" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Properties
{
  Properties() = default;

  template<typename Node, typename Scope>
  explicit Properties(const Node &, Scope &)
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PROPERTIES_HPP_
