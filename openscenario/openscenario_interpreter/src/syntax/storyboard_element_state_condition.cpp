// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
StoryboardElementStateCondition::StoryboardElementStateCondition(
  const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  storyboard_element_ref(readAttribute<String>("storyboardElementRef", node, local())),
  storyboard_element_type(
    readAttribute<StoryboardElementType>("storyboardElementType", node, local())),
  state(readAttribute<StoryboardElementState>("state", node, local())),
  current_state(StoryboardElementState::standbyState)
{
  /*
     Register a callback for the StoryboardElement specified by
     storyboardElementRef to notify that it has transitioned to the monitored
     state.

     Note that there is no guarantee that the StoryboardElement will remain in
     the same state as when the notification is made until
     StoryboardElementStateCondition::evaluate is executed after the callback
     is called. In other words, the StoryboardElement may immediately
     transition to the next state after calling the callback function.
  */

  auto register_callback = [this]() {
    local()
      .ref<StoryboardElement>(storyboard_element_ref)
      .addTransitionCallback(state, [this](auto && storyboard_element) {
        current_state = storyboard_element.state().template as<StoryboardElementState>();
      });
  };

  Storyboard::thunks.push(register_callback);
}

auto StoryboardElementStateCondition::description() const -> String
{
  std::stringstream description;

  description << "The state of StoryboardElement " << std::quoted(storyboard_element_ref)
              << " (= " << current_state << ") is given state " << state << "?";

  return description.str();
}

auto StoryboardElementStateCondition::evaluate() -> Object
{
  auto update = [this]() {
    auto storyboard_element = [this]() {
      return local().ref<StoryboardElement>(storyboard_element_ref);
    };
    return current_state = storyboard_element().state().template as<StoryboardElementState>();
  };

  /*
     Note that current_state may have been updated by a callback function set
     in the constructor (before this member function was called).  And at this
     point local().ref<StoryboardElement>(storyboard_element_ref).state() may
     have transitioned to a different state than the one recorded in
     current_state.

     Therefore, we must first check to see if the callback function has updated
     current_state (= has the StoryboardElement transitioned to the monitored
     state at least once since the last time evaluate was called), and then
     check to see if the current StoryboardElement state is indeed the state
     being monitored.
  */
  return asBoolean(current_state == state or update() == state);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
