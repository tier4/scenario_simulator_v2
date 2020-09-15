#ifndef SCENARIO_RUNNER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_

#include <scenario_runner/syntax/rule.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== SimulationTimeCondition ==============================================
   *
   * <xsd:complexType name="SimulationTimeCondition">
   *   <xsd:attribute name="value" type="Double" use="required"/>
   *   <xsd:attribute name="rule" type="Rule" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct SimulationTimeCondition
  {
    const Double value;

    const Rule compare;

    auto begin() const
    {
      static const auto time { std::chrono::high_resolution_clock::now() };
      return time;
    }

    template <typename Node, typename Scope>
    explicit SimulationTimeCondition(const Node& node, Scope& scope)
      : value { readAttribute<Double>(node, scope, "value") }
      , compare { readAttribute<Rule>(node, scope, "rule") }
    {
      begin();
    }

    auto evaluate() const
    {
      const auto simulation_time {
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::high_resolution_clock::now() - begin()
          ).count()
      };

      const auto specified_time {
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::seconds(value)
          ).count()
      };

      const auto result { compare(simulation_time, specified_time) ? true_v : false_v };

      std::cout << indent << "SimulationTime [" << simulation_time << " is " << compare << " " << specified_time << "? => " << result << "]" << std::endl;

      return result;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
