#ifndef SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_

#include <scenario_runner/syntax/speed_action.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== LongitudinalAction ===================================================
   *
   * <xsd:complexType name="LongitudinalAction">
   *   <xsd:choice>
   *     <xsd:element name="SpeedAction" type="SpeedAction"/>
   *     <xsd:element name="LongitudinalDistanceAction" type="LongitudinalDistanceAction"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct LongitudinalAction
    : public Object
  {
    template <typename Node, typename Scope>
    explicit LongitudinalAction(const Node& node, Scope& scope)
    {
      callWithElements(node, "SpeedAction", 0, 1, [&](auto&& node)
      {
        return rebind<SpeedAction>(node, scope);
      });

      callWithElements(node, "LongitudinalDistanceAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    }

    using Object::evaluate;
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_
