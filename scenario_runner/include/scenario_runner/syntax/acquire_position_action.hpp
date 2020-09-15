#ifndef SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_

#include <scenario_runner/syntax/position.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== AcquirePositionAction ================================================
   *
   * <xsd:complexType name="AcquirePositionAction">
   *   <xsd:all>
   *     <xsd:element name="Position" type="Position"/>
   *   </xsd:all>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct AcquirePositionAction
  {
    Scope inner_scope;

    const Position position;

    template <typename Node>
    explicit AcquirePositionAction(const Node& node, Scope& outer_scope)
      : inner_scope { outer_scope }
      , position    { readElement<Position>("Position", node, inner_scope) }
    {}

    std::unordered_map<std::string, Boolean> accomplishments;

    auto start()
    {
      accomplishments.clear();

      if (position.is<LanePosition>())
      {
        for (const auto& actor : inner_scope.actors)
        {
          accomplishments.emplace(actor, false);

          // inner_scope.connection->entity->requestAcquirePosition(
          //   actor,
          //   Integer(position.as<LanePosition>().lane_id),
          //   position.as<LanePosition>().s,
          //   position.as<LanePosition>().offset);
        }
      }
      else
      {
        THROW(ImplementationFault);
      }
    }

    auto accomplished()
    {
    #ifndef SCENARIO_RUNNER_NO_EXTENSION
      if (position.is<LanePosition>())
      {
        for (auto&& each : accomplishments)
        {
          if (not cdr(each))
          {
            // cdr(each) =
            //   inner_scope.connection->entity->reachPosition(
            //     car(each),
            //     Integer(position.as<LanePosition>().lane_id),
            //     position.as<LanePosition>().s,
            //     position.as<LanePosition>().offset,
            //     5.0);
          }
        }

        return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
      }
      else
      {
        THROW(ImplementationFault);
      }
    #else
      return true;
    #endif
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
