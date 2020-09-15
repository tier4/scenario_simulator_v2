#ifndef SCENARIO_RUNNER__SYNTAX__INIT_HPP_
#define SCENARIO_RUNNER__SYNTAX__INIT_HPP_

#include <scenario_runner/syntax/init_actions.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Init ================================================================
 *
 * <xsd:complexType name="Init">
 *   <xsd:sequence>
 *     <xsd:element name="Actions" type="InitActions"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Init
  : public InitActions
{
  template<typename Node, typename Scope>
  explicit Init(const Node & node, Scope & scope)
  : InitActions{readElement<InitActions>("Actions", node, scope)}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__INIT_HPP_
