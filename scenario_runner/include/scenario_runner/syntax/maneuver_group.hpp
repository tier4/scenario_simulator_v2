#ifndef SCENARIO_RUNNER__SYNTAX__MANEUVER_GROUP_HPP_
#define SCENARIO_RUNNER__SYNTAX__MANEUVER_GROUP_HPP_

#include <scenario_runner/syntax/actors.hpp>
#include <scenario_runner/syntax/maneuver.hpp>
#include <scenario_runner/syntax/storyboard_element.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== ManeuverGroup ========================================================
   *
   * <xsd:complexType name="ManeuverGroup">
   *   <xsd:sequence>
   *     <xsd:element name="Actors" type="Actors"/>
   *     <xsd:element name="CatalogReference" type="CatalogReference" minOccurs="0" maxOccurs="unbounded"/>
   *     <xsd:element name="Maneuver" type="Maneuver" minOccurs="0" maxOccurs="unbounded"/>
   *   </xsd:sequence>
   *   <xsd:attribute name="maximumExecutionCount" type="UnsignedInt" use="required"/>
   *   <xsd:attribute name="name" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct ManeuverGroup
    : public StoryboardElement<ManeuverGroup>
    , public Objects
  {
    const String name;

    Scope inner_scope;

    const Actors actors;

    template <typename Node, typename Scope>
    explicit ManeuverGroup(const Node& node, Scope& outer_scope)
      : StoryboardElement { readAttribute<UnsignedInteger>(node, outer_scope, "maximumExecutionCount", 1) }
      , name              { readAttribute<String>         (node, outer_scope, "name") }
      , inner_scope       { outer_scope }
      , actors            { readElement<Actors>("Actors", node, inner_scope) }
    {
      callWithElements(node, "CatalogReference", 0, unbounded, THROW_UNSUPPORTED_ERROR(node));

      callWithElements(node, "Maneuver", 0, unbounded, [&](auto&& node)
      {
        return makeStoryboardElement<Maneuver>(node, inner_scope);
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
     * ManeuverGroup
     *   A ManeuverGroup's goal is accomplished when all its Maneuvers are in
     *   the completeState.
     *
     * ---------------------------------------------------------------------- */
    auto accomplished() const
    {
      return std::all_of(std::begin(*this), std::end(*this), [&](auto&& each)
             {
               return each.template as<Maneuver>().complete();
             });
    }

    using StoryboardElement::evaluate;

    void stop()
    {
      for (auto&& each : *this)
      {
        each.as<Maneuver>().override();
        each.evaluate();
      }
    }

    void run()
    {
      for (auto&& each : *this)
      {
        each.evaluate();
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__MANEUVER_GROUP_HPP_
