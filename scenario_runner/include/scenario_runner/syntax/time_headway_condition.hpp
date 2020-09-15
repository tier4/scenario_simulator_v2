#ifndef SCENARIO_RUNNER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_

#include <scenario_runner/syntax/rule.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== TimeHeadwayCondition =================================================
   *
   * <xsd:complexType name="TimeHeadwayCondition">
   *   <xsd:attribute name="entityRef" type="String" use="required"/>
   *   <xsd:attribute name="value" type="Double" use="required"/>
   *   <xsd:attribute name="freespace" type="Boolean" use="required"/>
   *   <xsd:attribute name="alongRoute" type="Boolean" use="required"/>
   *   <xsd:attribute name="rule" type="Rule" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct TimeHeadwayCondition
  {
    const String entity_ref;

    const Double value;

    const Boolean freespace;

    const Boolean along_route;

    const Rule compare;

    Scope inner_scope;

    const TriggeringEntities trigger;

    template <typename Node>
    explicit TimeHeadwayCondition(const Node& node, Scope& outer_scope, const TriggeringEntities& trigger)
      : entity_ref  { readAttribute<String> (node, outer_scope, "entityRef") }
      , value       { readAttribute<Double> (node, outer_scope, "value") }
      , freespace   { readAttribute<Boolean>(node, outer_scope, "freespace") }
      , along_route { readAttribute<Boolean>(node, outer_scope, "alongRoute") }
      , compare     { readAttribute<Rule>   (node, outer_scope, "rule") }
      , inner_scope { outer_scope }
      , trigger     { trigger }
    {}

    auto evaluate()
    {
      // return
      //   asBoolean(
      //     trigger([&](auto&& entity)
      //     {
      //       return compare(inner_scope.getTimeHeadway(entity, entity_ref), value);
      //     }));

      return false_v;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
