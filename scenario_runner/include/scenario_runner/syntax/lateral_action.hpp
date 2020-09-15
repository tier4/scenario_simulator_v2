#ifndef SCENARIO_RUNNER__SYNTAX__LATERAL_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LATERAL_ACTION_HPP_

#include <scenario_runner/syntax/lane_change_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== LateralAction ========================================================
 *
 * <xsd:complexType name="LateralAction">
 *   <xsd:choice>
 *     <xsd:element name="LaneChangeAction" type="LaneChangeAction"/>
 *     <xsd:element name="LaneOffsetAction" type="LaneOffsetAction"/>
 *     <xsd:element name="LateralDistanceAction" type="LateralDistanceAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct LateralAction
  : public Object
{
  template<typename Node, typename Scope>
  explicit LateralAction(const Node & node, Scope & scope)
  {
    callWithElements(node, "LaneChangeAction", 0, 1, [&](auto && node)
      {
        return rebind<LaneChangeAction>(node, scope);
      });

    callWithElements(node, "LaneOffsetAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "LateralDistanceAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__LATERAL_ACTION_HPP_
