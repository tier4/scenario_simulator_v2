#ifndef SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_

#include <scenario_runner/syntax/relative_distance_type.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== RelativeDistanceCondition ============================================
   *
   * <xsd:complexType name="RelativeDistanceCondition">
   *   <xsd:attribute name="entityRef" type="String" use="required"/>
   *   <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType" use="required"/>
   *   <xsd:attribute name="value" type="Double" use="required"/>
   *   <xsd:attribute name="freespace" type="Boolean" use="required"/>
   *   <xsd:attribute name="rule" type="Rule" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct RelativeDistanceCondition
  {
    const String entity_ref;

    const RelativeDistanceType relative_distance_type;

    const Double value;

    const Boolean freespace;

    const Rule compare;

    Scope inner_scope;

    const TriggeringEntities verify;

    template <typename Node, typename Scope>
    explicit RelativeDistanceCondition(const Node& node, Scope& outer_scope, const TriggeringEntities& triggering_entities)
      : entity_ref             { readAttribute<String>              (node, outer_scope, "entityRef") }
      , relative_distance_type { readAttribute<RelativeDistanceType>(node, outer_scope, "relativeDistanceType") }
      , value                  { readAttribute<Double>              (node, outer_scope, "value") }
      , freespace              { readAttribute<Boolean>             (node, outer_scope, "freespace") }
      , compare                { readAttribute<Rule>                (node, outer_scope, "rule") }
      , inner_scope            { outer_scope }
      , verify                 { triggering_entities }
    {}

    auto evaluate()
    {
      // TODO USE PARAMETER 'freespace'

      // switch (relative_distance_type)
      // {
      // case RelativeDistanceType::longitudinal:
      //
      //   return
      //     asBoolean(
      //       verify([&](auto&& subject)
      //       {
      //         const auto distance { std::fabs(inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.x) };
      //         std::cout << "DISTANCE: " << distance << std::endl;
      //         return compare(distance, value);
      //       }));
      //
      // case RelativeDistanceType::lateral:
      //
      //   return
      //     asBoolean(
      //       verify([&](auto&& subject)
      //       {
      //         const auto distance { std::fabs(inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.y) };
      //         std::cout << "DISTANCE: " << distance << std::endl;
      //         return compare(distance, value);
      //       }));
      //
      // case RelativeDistanceType::cartesianDistance:
      //
      //   return
      //     asBoolean(
      //       verify([&](auto&& subject)
      //       {
      //         const auto distance {
      //           std::hypot(
      //             inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.x,
      //             inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.y)
      //         };
      //         std::cout << "DISTANCE: " << distance << std::endl;
      //         return compare(distance, value);
      //       }));
      //
      // default:
      //   THROW(ImplementationFault);
      // }

      return false_v;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
