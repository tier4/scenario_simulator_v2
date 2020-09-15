#ifndef SCENARIO_RUNNER__SYNTAX__GLOBAL_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__GLOBAL_ACTION_HPP_

#include <scenario_runner/syntax/infrastructure_action.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== GlobalAction =========================================================
   *
   * <xsd:complexType name="GlobalAction">
   *   <xsd:choice>
   *     <xsd:element name="EnvironmentAction" type="EnvironmentAction"/>
   *     <xsd:element name="EntityAction" type="EntityAction"/>
   *     <xsd:element name="ParameterAction" type="ParameterAction"/>
   *     <xsd:element name="InfrastructureAction" type="InfrastructureAction"/>
   *     <xsd:element name="TrafficAction" type="TrafficAction"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct GlobalAction
    : public Object
  {
    template <typename Node, typename Scope>
    explicit GlobalAction(const Node& node, Scope&)
    {
      callWithElements(node, "EnvironmentAction",    0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "EntityAction",         0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "ParameterAction",      0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "InfrastructureAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "TrafficAction",        0, 1, THROW_UNSUPPORTED_ERROR(node));
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__GLOBAL_ACTION_HPP_
