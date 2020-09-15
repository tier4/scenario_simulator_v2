#ifndef SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_
#define SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_

#include <scenario_runner/syntax/absolute_target_lane.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== LaneChangeTarget =====================================================
   *
   * <xsd:complexType name="LaneChangeTarget">
   *   <xsd:choice>
   *     <xsd:element name="RelativeTargetLane" type="RelativeTargetLane"/>
   *     <xsd:element name="AbsoluteTargetLane" type="AbsoluteTargetLane"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct LaneChangeTarget
    : public Object
  {
    template <typename Node, typename... Ts>
    explicit LaneChangeTarget(const Node& node, Ts&&... xs)
    {
      callWithElements(node, "RelativeTargetLane", 0, 1, THROW_UNSUPPORTED_ERROR(node));

      callWithElements(node, "AbsoluteTargetLane", 0, 1, [&](auto&& node)
      {
        return rebind<AbsoluteTargetLane>(node, std::forward<decltype(xs)>(xs)...);
      });
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_

