#ifndef SCENARIO_RUNNER__SYNTAX__CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__CONDITION_HPP_

#include <scenario_runner/syntax/by_entity_condition.hpp>
#include <scenario_runner/syntax/by_value_condition.hpp>
#include <scenario_runner/syntax/condition_edge.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Condition ============================================================
   *
   * <xsd:complexType name="Condition">
   *   <xsd:choice>
   *     <xsd:element name="ByEntityCondition" type="ByEntityCondition"/>
   *     <xsd:element name="ByValueCondition" type="ByValueCondition"/>
   *   </xsd:choice>
   *   <xsd:attribute name="name" type="String" use="required"/>
   *   <xsd:attribute name="delay" type="Double" use="required"/>
   *   <xsd:attribute name="conditionEdge" type="ConditionEdge" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Condition
    : public Object
  {
    const String name;

    const Double delay;

    const ConditionEdge condition_edge;

    template <typename Node, typename Scope>
    explicit Condition(const Node& node, Scope& scope)
      : name { readAttribute<String>(node, scope, "name") }
      , delay { readAttribute<Double>(node, scope, "delay", 0) }
      , condition_edge { readAttribute<ConditionEdge>(node, scope, "conditionEdge") }
    {
      callWithElements(node, "ByEntityCondition", 0, 1, [&](auto&& node)
      {
        return rebind<ByEntityCondition>(node, scope);
      });

      callWithElements(node, "ByValueCondition", 0, 1, [&](auto&& node)
      {
        return rebind<ByValueCondition>(node, scope);
      });
    }

    decltype(auto) evaluate() const
    {
      std::cout << (indent++) << "Condition " << cyan << "\"" << name << "\"" << console::reset << std::endl;

      BOOST_SCOPE_EXIT_ALL()
      {
        --indent;
      };

      return Object::evaluate();
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CONDITION_HPP_
