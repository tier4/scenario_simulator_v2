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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/priority.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Event ------------------------------------------------------------------
 *
 *  <xsd:complexType name="Event">
 *    <xsd:sequence>
 *      <xsd:element name="Action" maxOccurs="unbounded" type="Action"/>
 *      <xsd:element name="StartTrigger" type="Trigger"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="priority" type="Priority" use="required"/>
 *    <xsd:attribute name="maximumExecutionCount" type="UnsignedInt"/>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Event : private Scope, public StoryboardElement<Event>
{
  const String name;  // Name of the event.

  const Priority priority;  // Priority of each event.

  Elements actions;

  Trigger start_trigger;

  template <typename XML>
  explicit Event(const XML & node, Scope & outer_scope)
  : Scope(outer_scope),
    StoryboardElement(
      readAttribute<UnsignedInt>("maximumExecutionCount", node, localScope(), UnsignedInt(1))),
    name(readAttribute<String>("name", node, localScope())),
    priority(readAttribute<Priority>("priority", node, localScope())),
    start_trigger(readElement<Trigger>("StartTrigger", node, localScope()))
  {
    callWithElements(node, "Action", 1, unbounded, [&](auto && node) {
      return actions.push_back(readStoryboardElement<Action>(node, localScope()));
    });
  }

  auto ready() { return start_trigger.evaluate().as<Boolean>(); }

  static constexpr auto stopTriggered() noexcept { return false; }

  /* -------------------------------------------------------------------------
   *
   *  An Event's goal is accomplished when all its Actions are in the
   *  completeState.
   *
   * ---------------------------------------------------------------------- */
  auto accomplished() const
  {
    return std::all_of(std::begin(actions), std::end(actions), [](auto && each) {
      return each.template as<Action>().complete();
    });
  }

  void start()
  {
    for (auto && each : actions) {
      each.as<Action>().changeStateIf(true, standby_state);
    }
  }

  void stop()
  {
    for (auto && each : actions) {
      each.as<Action>().override();
      each.evaluate();
    }
  }

  void run()
  {
    for (auto && action : actions) {
      action.evaluate();
    }
  }
};

nlohmann::json & operator<<(nlohmann::json &, const Event &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__EVENT_HPP_
