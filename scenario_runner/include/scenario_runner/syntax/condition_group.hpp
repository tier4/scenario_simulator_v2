#ifndef SCENARIO_RUNNER__SYNTAX__CONDITION_GROUP_HPP_
#define SCENARIO_RUNNER__SYNTAX__CONDITION_GROUP_HPP_

#include <scenario_runner/syntax/condition.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== ConditionGroup =======================================================
   *
   * A condition group is an association of conditions that is assessed during
   * simulation time and signals true when all associated conditions are
   * evaluated to true.
   *
   * <xsd:complexType name="ConditionGroup">
   *   <xsd:sequence>
   *     <xsd:element name="Condition" type="Condition" maxOccurs="unbounded"/>
   *   </xsd:sequence>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct ConditionGroup
    : public std::vector<Condition>
  {
    template <typename Node, typename Scope>
    explicit ConditionGroup(const Node& node, Scope& scope)
    {
      callWithElements(node, "Condition", 1, unbounded, [&](auto&& node)
      {
        emplace_back(node, scope);
      });
    }

    auto evaluate() const
    {
      return
        asBoolean(
          std::all_of(std::begin(*this), std::end(*this), [&](auto&& each)
          {
            return each.evaluate().template as<Boolean>(__FILE__, __LINE__);
          }));
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CONDITION_GROUP_HPP_
