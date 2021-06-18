// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- StoryboardElementStateCondition ----------------------------------------
 *
 *  <xsd:complexType name="StoryboardElementStateCondition">
 *    <xsd:attribute name="storyboardElementType" type="StoryboardElementType" use="required"/>
 *    <xsd:attribute name="storyboardElementRef" type="String" use="required"/>
 *    <xsd:attribute name="state" type="StoryboardElementState" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct StoryboardElementStateCondition : private Scope
{
  const String name;

  const StoryboardElementType type;

  const StoryboardElementState state;

  template <typename Node>
  explicit StoryboardElementStateCondition(const Node & node, const Scope & outer_scope)
  : Scope(outer_scope),
    name(readAttribute<String>("storyboardElementRef", node, localScope())),
    type(readAttribute<StoryboardElementType>("storyboardElementType", node, localScope())),
    state(readAttribute<StoryboardElementState>("state", node, localScope()))
  {
  }

  auto compare(const Element & lhs, StoryboardElementState rhs) const
  {
    return asBoolean(lhs.as<StoryboardElementState>() == rhs);
  }

  auto evaluate() const
  {
    const auto result = compare(localScope().storyboard_elements.at(name).currentState(), state);

#ifndef NDEBUG
    std::cout << indent << "StoryboardElementState [Is " << cyan << "\"" << name << "\"" << reset
              << " in " << state << "? => " << result << "]" << std::endl;
#endif

    return result;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_CONDITION_HPP_
