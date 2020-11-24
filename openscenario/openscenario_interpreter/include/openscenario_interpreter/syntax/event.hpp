// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_

#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/priority.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== Event ================================================================
 *
 * <xsd:complexType name="Event">
 *   <xsd:sequence>
 *     <xsd:element name="Action" maxOccurs="unbounded" type="Action"/>
 *     <xsd:element name="StartTrigger" type="Trigger"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="priority" type="Priority" use="required"/>
 *   <xsd:attribute name="maximumExecutionCount" type="UnsignedInt"/>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Event
  : public StoryboardElement<Event>, public Elements
{
  // Name of the event.
  const String name;

  // Priority of each event.
  const Priority priority;

  Scope inner_scope;

  Element start_trigger;

  template<typename Node, typename Scope>
  explicit Event(const Node & node, Scope & outer_scope)
  : StoryboardElement{
      readAttribute<UnsignedInt>("maximumExecutionCount", node, outer_scope, UnsignedInt(1))},
    name{readAttribute<String>("name", node, outer_scope)},
    priority{readAttribute<Priority>("priority", node, outer_scope)},
    inner_scope{outer_scope}
  {
    callWithElements(
      node, "Action", 1, unbounded, [&](auto && node)
      {
        return push_back(readStoryboardElement<Action>(node, inner_scope, maximum_execution_count));
      });

    callWithElements(
      node, "StartTrigger", 1, 1, [&](auto && node)
      {
        return start_trigger.rebind<Trigger>(node, inner_scope);
      });
  }

  auto ready() const
  {
    return start_trigger.evaluate().as<Boolean>(__FILE__, __LINE__);
  }

  static constexpr auto stopTriggered() noexcept
  {
    return false;
  }

  /* -------------------------------------------------------------------------
   *
   * Event
   *   An Event's goal is accomplished when all its Actions are in the
   *   completeState.
   *
   * ---------------------------------------------------------------------- */
  auto accomplished() const
  {
    return std::all_of(
      std::begin(*this), std::end(*this), [](auto && each)
      {
        return each.template as<Action>().complete();
      });
  }

  using StoryboardElement::evaluate;

  void stop()
  {
    for (auto && each : *this) {
      each.as<Action>().override ();
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
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
