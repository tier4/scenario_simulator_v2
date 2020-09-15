#ifndef SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_

#include <scenario_runner/syntax/acquire_position_action.hpp>
#include <scenario_runner/syntax/assign_route_action.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== RoutingAction ========================================================
   *
   * <xsd:complexType name="RoutingAction">
   *   <xsd:choice>
   *     <xsd:element name="AssignRouteAction" type="AssignRouteAction"/>
   *     <xsd:element name="FollowTrajectoryAction" type="FollowTrajectoryAction"/>
   *     <xsd:element name="AcquirePositionAction" type="AcquirePositionAction"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct RoutingAction
    : public Object
  {
    template <typename Node, typename Scope>
    explicit RoutingAction(const Node& node, Scope& outer_scope)
    {
      callWithElements(node, "AssignRouteAction", 0, 1, [&](auto&& node)
      {
        return rebind<AssignRouteAction>(node, outer_scope);
      });

      callWithElements(node, "FollowTrajectoryAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));

      callWithElements(node, "AcquirePositionAction", 0, 1, [&](auto&& node)
      {
        return rebind<AcquirePositionAction>(node, outer_scope);
      });
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ROUTING_ACTION_HPP_
