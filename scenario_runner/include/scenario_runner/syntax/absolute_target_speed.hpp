#ifndef SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_
#define SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== AbsoluteTargetSpeed ==================================================
   *
   * <xsd:complexType name="AbsoluteTargetSpeed">
   *   <xsd:attribute name="value" type="Double" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct AbsoluteTargetSpeed
  {
    const Double value;

    template <typename Node, typename Scope>
    explicit AbsoluteTargetSpeed(const Node& node, Scope& scope)
      : value { readAttribute<Double>(node, scope, "value") }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_

