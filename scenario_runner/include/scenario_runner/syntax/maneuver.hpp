#ifndef SCENARIO_RUNNER__SYNTAX__MANEUVER_HPP_
#define SCENARIO_RUNNER__SYNTAX__MANEUVER_HPP_

#include <scenario_runner/syntax/event.hpp>
#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/storyboard_element.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Maneuver =============================================================
 *
 * <xsd:complexType name="Maneuver">
 *   <xsd:sequence>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="Event" maxOccurs="unbounded" type="Event"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Maneuver
  : public StoryboardElement<Maneuver>,
  public Objects
{
  const String name;

  Scope inner_scope;

  template<typename Node, typename Scope>
  explicit Maneuver(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>(node, outer_scope, "name")},
    inner_scope{outer_scope}
  {
    callWithElements(node, "ParameterDeclarations", 0, 1, [&](auto && node)
      {
        return make<ParameterDeclarations>(node, inner_scope);
      });

    callWithElements(node, "Event", 1, unbounded, [&](auto && node)
      {
        return makeStoryboardElement<Event>(node, inner_scope);
      });
  }

  static constexpr auto ready() noexcept
  {
    return true;
  }

  static constexpr auto stopTriggered() noexcept
  {
    return false;
  }

  /* -------------------------------------------------------------------------
   *
   * Maneuver
   *   A Maneuver's goal is accomplished when all its Events are in the
   *   completeState.
   *
   * ---------------------------------------------------------------------- */
  auto accomplished() const
  {
    return std::all_of(std::begin(*this), std::end(*this), [](auto && each)
             {
               return each.template as<Event>().complete();
             });
  }

  using StoryboardElement::evaluate;

  void stop()
  {
    for (auto && each : *this) {
      each.as<Event>().override ();
      each.evaluate();
    }
  }

  void run()
  {
    for (auto && each : *this) {
      each.evaluate();
    }
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__MANEUVER_HPP_
