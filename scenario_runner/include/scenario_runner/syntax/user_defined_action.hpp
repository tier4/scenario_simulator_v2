#ifndef SCENARIO_RUNNER__SYNTAX__USER_DEFINED_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__USER_DEFINED_ACTION_HPP_

#include <scenario_runner/syntax/custom_command_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== UserDefinedAction ====================================================
 *
 * <xsd:complexType name="UserDefinedAction">
 *   <xsd:sequence>
 *     <xsd:element name="CustomCommandAction" type="CustomCommandAction"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct UserDefinedAction
  : public Object
{
  template<typename Node, typename Scope>
  explicit UserDefinedAction(const Node & node, Scope & scope)
  {
    callWithElements(node, "CustomCommandAction", 1, 1, [&](auto && node)
      {
        return rebind<CustomCommandAction>(node, scope);
      });
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__USER_DEFINED_ACTION_HPP_
