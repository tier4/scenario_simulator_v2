#ifndef SCENARIO_RUNNER__SYNTAX__TRIGGER_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRIGGER_HPP_

#include <chrono> // TODO REMOVE
#include <scenario_runner/syntax/condition_group.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Trigger ==============================================================
 *
 * <xsd:complexType name="Trigger">
 *   <xsd:sequence>
 *     <xsd:element name="ConditionGroup" type="ConditionGroup" minOccurs="0" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Trigger
  : public std::vector<ConditionGroup>
{
  template<typename Node, typename Scope>
  explicit Trigger(const Node & node, Scope & scope)
  {
    callWithElements(node, "ConditionGroup", 0, unbounded, [&](auto && node)
      {
        emplace_back(node, scope);
      });
  }

  auto evaluate() const
  {
    /* -----------------------------------------------------------------------
     *
     * A trigger is then defined as an association of condition groups. A
     * trigger evaluates to true if at least one of the associated condition
     * groups evaluates to true, otherwise it evaluates to false (OR
     * operation).
     *
     * -------------------------------------------------------------------- */
    return
      asBoolean(
      std::any_of(std::begin(*this), std::end(*this), [&](auto && each)
      {
        return each.evaluate().template as<Boolean>(__FILE__, __LINE__);
      }));
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRIGGER_HPP_
