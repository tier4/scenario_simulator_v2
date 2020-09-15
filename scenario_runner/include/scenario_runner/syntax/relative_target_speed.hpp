#ifndef SCENARIO_RUNNER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
#define SCENARIO_RUNNER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_

#include <scenario_runner/syntax/speed_target_value_type.hpp>
#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== RelativeTargetSpeed ==================================================
   *
   * <xsd:complexType name="RelativeTargetSpeed">
   *   <xsd:attribute name="entityRef" type="String" use="required"/>
   *   <xsd:attribute name="value" type="Double" use="required"/>
   *   <xsd:attribute name="speedTargetValueType" type="SpeedTargetValueType" use="required"/>
   *   <xsd:attribute name="continuous" type="Boolean" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct RelativeTargetSpeed
  {
    const String entity_ref;

    const Double value;

    const SpeedTargetValueType speed_target_value_type;

    const Boolean continuous;

    template <typename Node, typename Scope>
    explicit RelativeTargetSpeed(const Node& node, Scope& scope)
      : entity_ref { readAttribute<String>(node, scope, "entityRef") }
      , value { readAttribute<Double>(node, scope, "value") }
      , speed_target_value_type { readAttribute<SpeedTargetValueType>(node, scope, "speedTargetValueType", SpeedTargetValueType::delta) }
      , continuous { readAttribute<Boolean>(node, scope, "continuous", false) }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
