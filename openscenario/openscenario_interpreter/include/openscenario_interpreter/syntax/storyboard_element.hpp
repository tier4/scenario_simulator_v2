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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_

#include <boost/mpl/and.hpp>
#include <boost/scope_exit.hpp>
#include <concealer/conversion.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state.hpp>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <typename T>
class StoryboardElement
{
public:
  const std::size_t maximum_execution_count;

  std::size_t current_execution_count;

  Element current_state;

  explicit constexpr StoryboardElement(std::size_t maximum_execution_count = 1)
  : maximum_execution_count(maximum_execution_count),
    current_execution_count(0),
    current_state(standby_state)
  {
  }

  const auto & state() const { return current_state; }

#define BOILERPLATE(NAME, STATE)                                                           \
  constexpr auto NAME() const noexcept                                                     \
  {                                                                                        \
    return state().template as<StoryboardElementState>() == StoryboardElementState::STATE; \
  }                                                                                        \
  static_assert(true, "")

  BOILERPLATE(standby, standbyState);
  BOILERPLATE(starting, startTransition);
  BOILERPLATE(running, runningState);
  BOILERPLATE(ending, endTransition);
  BOILERPLATE(complete, completeState);
  BOILERPLATE(stopping, stopTransition);
  BOILERPLATE(skipping, skipTransition);

#undef BOILERPLATE

  template <typename Boolean, REQUIRES(std::is_convertible<Boolean, bool>)>
  auto changeStateIf(
    Boolean && test, const Element & consequent_state, const Element & alternate_state)
  {
    if (test) {
#ifndef NDEBUG
      std::cout << indent;
      std::cout << typeid(T).name();
      std::cout << "::evaluate [" << current_state << " => " << consequent_state << "]";
      std::cout << std::endl;
#endif
      return current_state = consequent_state;
    } else {
      return current_state = alternate_state;
    }
  }

  template <typename Boolean, REQUIRES(std::is_convertible<Boolean, bool>)>
  decltype(auto) changeStateIf(Boolean && test, const Element & consequent_state)
  {
    return changeStateIf(test, consequent_state, current_state);
  }

  template <typename Predicate, typename... Ts, REQUIRES(std::is_function<Predicate>)>
  decltype(auto) changeStateIf(Predicate && predicate, Ts &&... xs)
  {
    return changeStateIf(predicate(), std::forward<decltype(xs)>(xs)...);
  }

  Element override()
  {
#ifndef NDEBUG
    std::cout << state() << std::endl;
#endif
    if (!complete() && !stopping()) {
      return current_state = stop_transition;
    } else {
      return current_state;
    }
  }

private:
  static constexpr void start() noexcept {}

#define DEFINE_PERFECT_FORWARD(IDENTIFIER, CONST)                                       \
  template <typename... Ts>                                                             \
  constexpr decltype(auto) IDENTIFIER(Ts &&... xs) CONST                                \
  {                                                                                     \
    return static_cast<CONST T &>(*this).IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
  }                                                                                     \
  static_assert(true, "")

  DEFINE_PERFECT_FORWARD(accomplished, const);
  DEFINE_PERFECT_FORWARD(ready, );
  DEFINE_PERFECT_FORWARD(run, );
  DEFINE_PERFECT_FORWARD(stop, );
  DEFINE_PERFECT_FORWARD(stopTriggered, );

#undef DEFINE_PERFECT_FORWARD

protected:
  auto rename(const std::string & name) const
  {
    static std::size_t id = 0;
    return name.empty() ? std::string("annonymous-") + std::to_string(++id) : name;
  }

  std::unordered_set<std::string> names;

  auto unique(const std::string & name) { return cdr(names.emplace(name)); }

  template <typename U, typename Node, typename Scope, typename... Ts>
  decltype(auto) readStoryboardElement(const Node & node, Scope & inner_scope, Ts &&... xs)
  {
    const auto name = rename(readAttribute<String>("name", node, inner_scope));

    if (unique(name)) {
      return inner_scope.storyboard_elements[name] =
               make<U>(node, inner_scope, std::forward<decltype(xs)>(xs)...);
    } else {
      std::stringstream ss{};
      ss << "detected redefinition of StoryboardElement named \'" << name << "\' ";
      ss << "(class " << typeid(U).name() << ")";
      throw SyntaxError(ss.str());
    }
  }

public:
  /* ---- States and Transitions of StoryboardElements -------------------------
   *
   *  See https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_states_and_transitions_of_storyboardelements
   *
   * ------------------------------------------------------------------------ */
  auto evaluate()
  {
    if (stopTriggered()) {
      override();
    }

#ifndef NDEBUG
    std::cout << (indent++);
    std::cout << "- evaluate: \x1b[36m";
    std::cout << std::quoted(static_cast<const T &>(*this).name);
    std::cout << "\x1b[0m";
    std::cout << " [" << current_state << "] ";
    std::cout << std::endl;
#endif

    BOOST_SCOPE_EXIT_ALL() { --indent; };

    switch (state().template as<StoryboardElementState>(__FILE__, __LINE__)) {
      /* ---- StandBy ----------------------------------------------------------
       *
       *  This is the default initialization state of a StoryboardElement. When
       *  it is in this state, the runtime instantiation of the
       *  StoryboardElement is ready to execute once given a startTrigger. A
       *  runtime instantiation of any StoryboardElement is created once its
       *  parent element is in the standbyState. From the standbyState, the
       *  Story element instantaneously transitions into the runningState.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::standbyState:
        return changeStateIf(ready(), start_transition);

      /* ---- Start ------------------------------------------------------------
       *
       *  The startTransition symbolizes that the execution of the runtime
       *  instantiation is now starting. The startTransition can be used in
       *  conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::startTransition:
        static_cast<T &>(*this).start();  // NOTE: DON'T USE MACRO DEFINE_PERFECT_FORWARD FOR THIS!
        ++current_execution_count;
        return changeStateIf(std::true_type(), running_state);

      /* ---- Running ----------------------------------------------------------
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
       *    A ManeuverGroup's goal is accomplished when all its Maneuvers are in
       *    the completeState.
       *
       *  Act
       *    An Act's goal is accomplished when all its ManeuverGroups are in the
       *    completeState.
       *
       *  Story
       *    A Story's goal is accomplished when all its Acts are in the
       *    completeState.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::runningState:
        run();
        return changeStateIf(accomplished(), end_transition);

      /* ---- End --------------------------------------------------------------
       *
       *  The endTransition occurs when the runtime instantiation of the
       *  StoryboardElement accomplishes its goal. Once the endTransition occurs,
       *  a check for completeness is made. A positive outcome moves the state
       *  machine to the completeState, whereas a negative outcome moves the
       *  state machine to the standbyState. The endTransition can be used in
       *  conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::endTransition:
        return changeStateIf(
          current_execution_count < maximum_execution_count, standby_state, complete_state);

      /* ---- Complete ---------------------------------------------------------
       *
       *  The completeState signals that the runtime instantiation of the
       *  StoryboardElement cannot reach a running state without external
       *  interference. If the affected runtime instantiation of the
       *  StoryboardElement is defined with a maximumExecutionCount, to be
       *  complete implies that there are no more executions left to run, or a
       *  stopTransition has occurred.
       *
       *  Checking for completeness involves verifying if the given runtime
       *  instantiation of the StoryboardElement still has executions left upon
       *  finishing the runningState. This check returns false if there are
       *  executions left. This check returns true if there are no executions
       *  left, or if the maximumExecutionCount is not defined in the
       *  StoryboardElement.
       *
       *  Resetting the completeState can only be achieved externally by the
       *  parent StoryboardElement whose child is in the completeState. This may
       *  only occur if the parent initiates a new execution.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::completeState:
        return current_state;

      /* ---- Skip -------------------------------------------------------------
       *
       *  Transition marking the moment an element is asked to move to the
       *  runningState but is instead skipped so it remains in the standbyState
       *  (only for Event instances). The skipTransition can be used in
       *  conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::skipTransition:
        return current_state;

      /* ---- Stop -------------------------------------------------------------
       *
       *  The stopTransition marks the reception of a stopTrigger or the
       *  storyboard element is overridden (applicable for Event and Action).
       *  This implies that the stopTransition cannot be reached other than with
       *  an external intervention to the runtime instantiation of the
       *  StoryboardElement.
       *
       *  When a runtime instantiation of a StoryboardElement goes through a
       *  stopTransition, all of its child elements are also forced to go
       *  through the same transition. The stopTransition can be used in
       *  conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      default:
      case StoryboardElementState::stopTransition:
        if (!accomplished()) {
          stop();
          return current_state;
        } else {
#ifndef NDEBUG
          std::cout << indent;
          std::cout << typeid(T).name();
          std::cout << "::stop [" << current_state << " => " << complete_state << "]";
          std::cout << std::endl;
#endif
          return current_state = complete_state;
        }
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_HPP_
