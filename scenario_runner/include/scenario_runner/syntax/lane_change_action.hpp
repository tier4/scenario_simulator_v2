#ifndef SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_ACTION_HPP_

#include <chrono>
#include <scenario_runner/syntax/lane_change_target.hpp>
#include <scenario_runner/syntax/transition_dynamics.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== LaneChangeAction =====================================================
   *
   * <xsd:complexType name="LaneChangeAction">
   *   <xsd:all>
   *     <xsd:element name="LaneChangeActionDynamics" type="TransitionDynamics"/>
   *     <xsd:element name="LaneChangeTarget" type="LaneChangeTarget"/>
   *   </xsd:all>
   *   <xsd:attribute name="targetLaneOffset" type="Double" use="optional"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct LaneChangeAction
  {
    const Double target_lane_offset;

    Scope inner_scope;

    const TransitionDynamics lane_change_action_dynamics;

    const LaneChangeTarget lane_change_target;

    template <typename Node>
    explicit LaneChangeAction(const Node& node, Scope& outer_scope)
      : target_lane_offset          { readAttribute<Double>(node, outer_scope, "targetLaneOffset", 0) }
      , inner_scope                 { outer_scope }
      , lane_change_action_dynamics { readElement<TransitionDynamics>("LaneChangeActionDynamics", node, inner_scope) }
      , lane_change_target          { readElement<LaneChangeTarget>  ("LaneChangeTarget",         node, inner_scope) }
    {}

    std::unordered_map<std::string, Boolean> accomplishments;

    void start()
    {
      accomplishments.clear();

      if (lane_change_target.is<AbsoluteTargetLane>())
      {
        for (const auto& actor : inner_scope.actors)
        {
          accomplishments.emplace(actor, false);

          // inner_scope.connection->entity->requestLaneChange(
          //   actor,
          //   Integer(lane_change_target.as<AbsoluteTargetLane>().value));
        }
      }
      else
      {
        THROW(ImplementationFault);
      }
    }

    auto accomplished()
    {
      if (lane_change_target.is<AbsoluteTargetLane>())
      {
        for (auto&& each : accomplishments)
        {
          if (not cdr(each))
          {
            // cdr(each) =
            //   inner_scope.connection->entity->isInLanelet(
            //     car(each),
            //     Integer(lane_change_target.as<AbsoluteTargetLane>().value));
          }
        }

        return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
      }
      else
      {
        THROW(ImplementationFault);
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_ACTION_HPP_

