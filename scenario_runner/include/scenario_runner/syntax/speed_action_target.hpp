#ifndef SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_TARGET_HPP_
#define SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_TARGET_HPP_

#include <scenario_runner/syntax/relative_target_speed.hpp>
#include <scenario_runner/syntax/absolute_target_speed.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== SpeedActionTarget ====================================================
 *
 * <xsd:complexType name="SpeedActionTarget">
 *   <xsd:choice>
 *     <xsd:element name="RelativeTargetSpeed" type="RelativeTargetSpeed"/>
 *     <xsd:element name="AbsoluteTargetSpeed" type="AbsoluteTargetSpeed"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct SpeedActionTarget
  : public Object
{
  template<typename Node, typename Scope>
  explicit SpeedActionTarget(const Node & node, Scope & scope)
  {
    callWithElements(node, "RelativeTargetSpeed", 0, 1, [&](auto && node)
      {
        return rebind<RelativeTargetSpeed>(node, scope);
      });

    callWithElements(node, "AbsoluteTargetSpeed", 0, 1, [&](auto && node)
      {
        return rebind<AbsoluteTargetSpeed>(node, scope);
      });
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_TARGET_HPP_
