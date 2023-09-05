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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_

#include <cstddef>
#include <limits>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
class StoryboardElement : private SimulatorCore::ConditionEvaluation
{
protected:
  Trigger stop_trigger;

  const std::size_t maximum_execution_count = 1;

  std::size_t current_execution_count = 0;

  Object current_state = standby_state;

  Elements elements;

  Trigger start_trigger{{ConditionGroup()}};

private:
  std::unordered_map<
    StoryboardElementState::value_type, std::vector<std::function<void(const StoryboardElement &)>>>
    callbacks;

public:
  // Storyboard
  explicit StoryboardElement(const Trigger & stop_trigger)  //
  : stop_trigger(stop_trigger)
  {
  }

  // Act
  explicit StoryboardElement(const Trigger & start_trigger, const Trigger & stop_trigger)
  : stop_trigger(stop_trigger), start_trigger(start_trigger)
  {
  }

  // Event
  explicit StoryboardElement(
    const std::size_t maximum_execution_count, const Trigger & start_trigger)
  : maximum_execution_count(maximum_execution_count), start_trigger(start_trigger)
  {
  }

  explicit StoryboardElement(const std::size_t maximum_execution_count = 1)
  : maximum_execution_count(maximum_execution_count)
  {
  }

  auto state() const -> const auto & { return current_state; }

  template <StoryboardElementState::value_type State>
  auto is() const
  {
    return current_state.as<StoryboardElementState>() == State;
  }

  auto override()
  {
    if (
      not is<StoryboardElementState::standbyState>() and
      not is<StoryboardElementState::stopTransition>()) {
      transitionTo(stop_transition);
    }
    return current_state;
  }

private:
  virtual auto accomplished() const -> bool
  {
    return std::all_of(std::begin(elements), std::end(elements), [](auto && element) {
      assert(element.template is_also<StoryboardElement>());
      return element.template as<StoryboardElement>()
        .template is<StoryboardElementState::completeState>();
    });
  }

  virtual auto run() -> void
  {
    for (auto && element : elements) {
      assert(element.is_also<StoryboardElement>());
      element.evaluate();
    }
  }

  virtual auto start() -> void {}

  virtual auto stop() -> void
  {
    for (auto && element : elements) {
      assert(element.is_also<StoryboardElement>());
      element.as<StoryboardElement>().override();
      element.evaluate();
    }
  }

protected:
  auto rename(const std::string & name) const
  {
    static std::size_t id = 0;
    return name.empty() ? std::string("anonymous-") + std::to_string(++id) : name;
  }

  std::unordered_set<std::string> names;

  auto unique(const std::string & name)
  {
    [[maybe_unused]] auto [iter, success] = names.emplace(name);
    return success;
  }

  template <typename U, typename Node, typename... Ts>
  auto readStoryboardElement(const Node & node, Scope & inner_scope, Ts &&... xs)
  {
    if (const auto name = rename(readAttribute<String>("name", node, inner_scope)); unique(name)) {
      auto element = make<U>(node, inner_scope, std::forward<decltype(xs)>(xs)...);
      inner_scope.insert(name, element);
      return element;
    } else {
      throw SyntaxError(
        "Detected redefinition of StoryboardElement named ", std::quoted(name), " (class ",
        makeTypename(typeid(U)), ")");
    }
  }

  template <typename U, typename Node, typename... Ts>
  auto readCatalogedStoryboardElement(const Node & node, Scope & inner_scope, Ts &&... xs)
  {
    if (auto element = CatalogReference(node, inner_scope).make();
        not unique(element.template as<U>().name)) {
      throw SyntaxError(
        "Detected redefinition of StoryboardElement named ",
        std::quoted(element.template as<U>().name), " (class ", makeTypename(typeid(U)), ")");
    } else {
      inner_scope.insert(element.template as<U>().name, element);
      return element;
    }
  }

public:
  void addTransitionCallback(
    StoryboardElementState::value_type transition,
    std::function<void(const StoryboardElement &)> callback)
  {
    callbacks[transition].push_back(callback);
  }

  auto transitionTo(const Object & state) -> bool
  {
    current_state = state;
    for (auto && callback : callbacks[current_state.as<StoryboardElementState>()]) {
      callback(std::as_const(*this));
    }
    return current_state == state;
  }

  virtual auto evaluate() -> Object
  {
    if (stop_trigger.evaluate().as<Boolean>()) {
      override();
    }

    // NOTE: https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_states_and_transitions_of_storyboardelements

    // NOTE: The fooTransition state must not return from case fooTransition
    // because it is a waypoint in the transition to barState.

  dispatch:
    switch (state().as<StoryboardElementState>()) {
      case StoryboardElementState::standbyState: /* ----------------------------
        *
        *  This is the default initialization state of a StoryboardElement.
        *  When it is in this state, the runtime instantiation of the
        *  StoryboardElement is ready to execute once given a startTrigger. A
        *  runtime instantiation of any StoryboardElement is created once its
        *  parent element is in the standbyState. From the standbyState, the
        *  Story element instantaneously transitions into the runningState.
        *
        * ------------------------------------------------------------------- */
        if (start_trigger.evaluate().as<Boolean>() and transitionTo(start_transition)) {
          goto dispatch;
        } else {
          return current_state;
        }

      case StoryboardElementState::startTransition: /* -------------------------
        *
        *  The startTransition symbolizes that the execution of the runtime
        *  instantiation is now starting. The startTransition can be used in
        *  conditions to trigger based on this transition.
        *
        * ------------------------------------------------------------------- */
        start();
        if (transitionTo(running_state)) ++current_execution_count;
        goto dispatch;

      case StoryboardElementState::runningState: /* ----------------------------
        *
        *  The runningState symbolizes that the execution of the runtime
        *  instantiation is now ongoing and has not yet accomplished its goal.
        *
        *  The concept of accomplishing a goal varies depending on the type of
        *  StoryboardElement under consideration:
        *
        *  Action
        *    An Action's goal is a function of the Action type and cannot be
        *    generalized. Accomplishing an Action's goal will involve meeting
        *    some arbitrary prerequisites related with the Action type (for
        *    example, a SpeedAction accomplishes its goal when the considered
        *    Entity is travelling at the prescribed speed). If an Action is
        *    acting on an EntitySelection, all instances of Entity within the
        *    selection have to complete in order to reach the completeState of
        *    the Action.
        *
        *  Event
        *    An Event's goal is accomplished when all its Actions are in the
        *    completeState.
        *
        *  Maneuver
        *    A Maneuver's goal is accomplished when all its Events are in the
        *    completeState.
        *
        *  ManeuverGroup
        *    A ManeuverGroup's goal is accomplished when all its Maneuvers are
        *    in the completeState.
        *
        *  Act
        *    An Act's goal is accomplished when all its ManeuverGroups are in
        *    the completeState.
        *
        *  Story
        *    A Story's goal is accomplished when all its Acts are in the
        *    completeState.
        *
        * ------------------------------------------------------------------- */
        if (run(), accomplished()) {
          transitionTo(end_transition);
          goto dispatch;
        } else {
          return current_state;
        }

      case StoryboardElementState::endTransition: /* ---------------------------
        *
        *  The endTransition occurs when the runtime instantiation of the
        *  StoryboardElement accomplishes its goal. Once the endTransition
        *  occurs, a check for completeness is made. A positive outcome moves
        *  the state machine to the completeState, whereas a negative outcome
        *  moves the state machine to the standbyState. The endTransition can
        *  be used in conditions to trigger based on this transition.
        *
        * -------------------------------------------------------------------- */
        transitionTo(
          current_execution_count < maximum_execution_count ? standby_state : complete_state);
        goto dispatch;

      case StoryboardElementState::completeState: /* ---------------------------
        *
        *  The completeState signals that the runtime instantiation of the
        *  StoryboardElement cannot reach a running state without external
        *  interference. If the affected runtime instantiation of the
        *  StoryboardElement is defined with a maximumExecutionCount, to be
        *  complete implies that there are no more executions left to run, or a
        *  stopTransition has occurred.
        *
        *  Checking for completeness involves verifying if the given runtime
        *  instantiation of the StoryboardElement still has executions left
        *  upon finishing the runningState. This check returns false if there
        *  are executions left. This check returns true if there are no
        *  executions left, or if the maximumExecutionCount is not defined in
        *  the StoryboardElement.
        *
        *  Resetting the completeState can only be achieved externally by the
        *  parent StoryboardElement whose child is in the completeState. This
        *  may only occur if the parent initiates a new execution.
        *
        * -------------------------------------------------------------------- */
        return current_state;

      case StoryboardElementState::skipTransition: /* --------------------------
        *
        *  Transition marking the moment an element is asked to move to the
        *  runningState but is instead skipped so it remains in the
        *  standbyState (only for Event instances). The skipTransition can be
        *  used in conditions to trigger based on this transition.
        *
        * ------------------------------------------------------------------- */
        throw Error("UNIMPLEMENTED!");

      default:
      case StoryboardElementState::stopTransition: /* --------------------------
        *
        *  The stopTransition marks the reception of a stopTrigger or the
        *  storyboard element is overridden (applicable for Event and Action).
        *  This implies that the stopTransition cannot be reached other than
        *  with an external intervention to the runtime instantiation of the
        *  StoryboardElement.
        *
        *  When a runtime instantiation of a StoryboardElement goes through a
        *  stopTransition, all of its child elements are also forced to go
        *  through the same transition. The stopTransition can be used in
        *  conditions to trigger based on this transition.
        *
        * ------------------------------------------------------------------- */
        if (not accomplished()) {
          stop();
        }

        transitionTo(complete_state);
        goto dispatch;
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_
