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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_

#include <open_scenario_interpreter/syntax/init.hpp>
#include <open_scenario_interpreter/syntax/story.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Storyboard ===========================================================
 *
 * <xsd:complexType name="Storyboard">
 *   <xsd:sequence>
 *     <xsd:element name="Init" type="Init"/>
 *     <xsd:element name="Story" maxOccurs="unbounded" type="Story"/>
 *     <xsd:element name="StopTrigger" type="Trigger"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Storyboard
  : public StoryboardElement<Storyboard>, public Elements
{
  Scope inner_scope;

  Init init;

  Trigger stop_trigger;

  const String name {"Storyboard"};  // XXX DIRTY HACK!!!

  template<typename Node, typename Scope>
  explicit Storyboard(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope},
    init{readElement<Init>("Init", node, inner_scope)},
    stop_trigger{readElement<Trigger>("StopTrigger", node, inner_scope)}
  {
    callWithElements(
      node, "Story", 1, unbounded, [&](auto && node)
      {
        return push_back(readStoryboardElement<Story>(node, inner_scope));
      });
  }

  auto ready() const noexcept
  {
    // return static_cast<bool>(inner_scope.connection);
    return true;
  }

  void start()
  {
    init.evaluate();   // NOTE RENAME TO 'start'?
  }

  decltype(auto) stopTriggered() const
  {
    return stop_trigger.evaluate().as<Boolean>();
  }

  void stop()
  {
    for (auto && each : *this) {
      each.as<Story>().override ();
      each.evaluate();
    }
  }

  auto accomplished() const
  {
    auto check =
      [](auto && each) {
        return each.template as<Story>().complete();
      };

    return std::all_of(std::begin(*this), std::end(*this), check);
  }

  auto run()
  {
    for (auto && story : *this) {
      story.evaluate();
    }
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
