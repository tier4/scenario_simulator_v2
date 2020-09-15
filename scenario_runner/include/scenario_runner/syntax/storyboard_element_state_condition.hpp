#ifndef SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_

#include <scenario_runner/syntax/storyboard_element_state.hpp>
#include <scenario_runner/syntax/storyboard_element_type.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== StoryboardElementStateCondition ======================================
 *
 * <xsd:complexType name="StoryboardElementStateCondition">
 *   <xsd:attribute name="storyboardElementType" type="StoryboardElementType" use="required"/>
 *   <xsd:attribute name="storyboardElementRef" type="String" use="required"/>
 *   <xsd:attribute name="state" type="StoryboardElementState" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct StoryboardElementStateCondition
{
  const String name;

  const StoryboardElementType type;

  const StoryboardElementState state;

  Scope inner_scope;

  template<typename Node, typename Scope>
  explicit StoryboardElementStateCondition(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>(node, outer_scope, "storyboardElementRef")},
    type{readAttribute<StoryboardElementType>(node, outer_scope, "storyboardElementType")},
    state{readAttribute<StoryboardElementState>(node, outer_scope, "state")},
    inner_scope{outer_scope}
  {}

  auto compare(const Object & lhs, StoryboardElementState rhs) const
  {
    return asBoolean(lhs.as<StoryboardElementState>() == rhs);
  }

  auto evaluate() const
  {
    const auto result {compare(inner_scope.storyboard_elements.at(name).currentState(), state)};

    std::cout << indent <<
      "StoryboardElementState [Is " <<
      cyan <<
      "\"" <<
      name <<
      "\"" <<
      reset <<
      " in " <<
      state <<
      "? => " <<
      result <<
      "]" <<
      std::endl;

    return result;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_
