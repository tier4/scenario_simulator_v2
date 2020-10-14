// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__STORY_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__STORY_HPP_

#include <open_scenario_interpreter/syntax/act.hpp>
#include <open_scenario_interpreter/syntax/storyboard_element.hpp>

#include <string>
#include <vector>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Story ================================================================
 *
 * <xsd:complexType name="Story">
 *   <xsd:sequence>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="Act" maxOccurs="unbounded" type="Act"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Story
  : public StoryboardElement<Story>, public Elements
{
  const String name;

  Scope inner_scope;

  template<typename Node>
  explicit Story(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>("name", node, outer_scope)},
    inner_scope{outer_scope}
  {
    callWithElements(
      node, "ParameterDeclarations", 0, 1, [&](auto && node)
      {
        return make<ParameterDeclarations>(node, inner_scope);
      });

    callWithElements(
      node, "Act", 1, unbounded, [&](auto && node)
      {
        return push_back(readStoryboardElement<Act>(node, inner_scope));
      });
  }

  static constexpr auto ready() noexcept
  {
    return true;
  }

  /* -------------------------------------------------------------------------
   *
   * Story
   *   A Story's goal is accomplished when all its Acts are in the
   *   completeState.
   *
   * ---------------------------------------------------------------------- */
  auto accomplished() const
  {
    return std::all_of(
      std::begin(*this), std::end(*this), [](auto && each)
      {
        return each.template as<Act>().complete();
      });
  }

  static constexpr auto stopTriggered() noexcept
  {
    return false;
  }

  auto stop()
  {
    for (auto && each : *this) {
      each.as<Act>().override ();
      each.evaluate();
    }
  }

  using StoryboardElement::evaluate;

  void run()
  {
    for (auto && act : *this) {
      act.evaluate();
    }
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__STORY_HPP_
