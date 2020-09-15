#ifndef SCENARIO_RUNNER__SYNTAX__ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ACTION_HPP_

#include <scenario_runner/syntax/global_action.hpp>
#include <scenario_runner/syntax/private_action.hpp>
#include <scenario_runner/syntax/storyboard_element.hpp>
#include <scenario_runner/syntax/user_defined_action.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Action ===============================================================
   *
   * <xsd:complexType name="Action">
   *   <xsd:choice>
   *     <xsd:element name="GlobalAction" type="GlobalAction"/>
   *     <xsd:element name="UserDefinedAction" type="UserDefinedAction"/>
   *     <xsd:element name="PrivateAction" type="PrivateAction"/>
   *   </xsd:choice>
   *   <xsd:attribute name="name" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Action
    : public StoryboardElement<Action>
    , public Object
  {
    const String name;

    template <typename Node, typename Scope>
    explicit Action(const Node& node, Scope& scope)
      : name { readAttribute<String>(node, scope, "name") }
    {
      callWithElements(node, "GlobalAction", 0, 1, [&](auto&& node)
      {
        return rebind<GlobalAction>(node, scope);
      });

      callWithElements(node, "UserDefinedAction", 0, 1, [&](auto&& node)
      {
        return rebind<UserDefinedAction>(node, scope);
      });

      callWithElements(node, "PrivateAction", 0, 1, [&](auto&& node)
      {
        return rebind<PrivateAction>(node, scope);
      });
    }

    auto ready() const noexcept(noexcept(static_cast<bool>(std::declval<Object&>())))
    {
      return static_cast<bool>(*this);
    }

    static constexpr auto stopTriggered() noexcept
    {
      return false;
    }

    using Object::start;

    /* -------------------------------------------------------------------------
     *
     * Action
     *   An Action's goal is a function of the Action type and cannot be
     *   generalized. Accomplishing an Action's goal will involve meeting some
     *   arbitrary prerequisites related with the Action type (for example, a
     *   SpeedAction accomplishes its goal when the considered Entity is
     *   travelling at the prescribed speed). If an Action is acting on an
     *   EntitySelection, all instances of Entity within the selection have to
     *   complete in order to reach the completeState of the Action.
     *
     * ---------------------------------------------------------------------- */
    using Object::accomplished;

    using StoryboardElement::evaluate;

    Boolean overridden { false };

    void stop()
    {
      if (overridden)
      {
        state = complete_state;
      }
      else
      {
        overridden = true;
      }
    }

    void run()
    {
      Object::evaluate();
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ACTION_HPP_
