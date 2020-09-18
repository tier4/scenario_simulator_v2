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

#ifndef SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_
#define SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_

#include <scenario_runner/object.hpp>

#include <string>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== StoryboardElementState ===============================================
 *
 * <xsd:simpleType name="StoryboardElementState">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="startTransition"/>
 *         <xsd:enumeration value="endTransition"/>
 *         <xsd:enumeration value="stopTransition"/>
 *         <xsd:enumeration value="skipTransition"/>
 *         <xsd:enumeration value="completeState"/>
 *         <xsd:enumeration value="runningState"/>
 *         <xsd:enumeration value="standbyState"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct StoryboardElementState
{
  enum value_type
  {
    /* ---- StandBy ----------------------------------------------------------
     *
     * This is the default initialization state of a StoryboardElement. When
     * it is in this state, the runtime instantiation of the StoryboardElement
     * is ready to execute once given a startTrigger. A runtime instantiation
     * of any StoryboardElement is created once its parent element is in the
     * standbyState. From the standbyState, the Story element instantaneously
     * transitions into the runningState.
     *
     * -------------------------------------------------------------------- */
    standbyState,

    /* ---- Running ----------------------------------------------------------
     *
     * The runningState symbolizes that the execution of the runtime
     * instantiation is now ongoing and has not yet accomplished its goal.
     *
     * The concept of accomplishing a goal varies depending on the type of
     * StoryboardElement under consideration:
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
     * Event
     *   An Event's goal is accomplished when all its Actions are in the
     *   completeState.
     *
     * Maneuver
     *   A Maneuver's goal is accomplished when all its Events are in the
     *   completeState.
     *
     * ManeuverGroup
     *   A ManeuverGroup's goal is accomplished when all its Maneuvers are in
     *   the completeState.
     *
     * Act
     *   An Act's goal is accomplished when all its ManeuverGroups are in the
     *   completeState.
     *
     * Story
     *   A Story's goal is accomplished when all its Acts are in the
     *   completeState.
     *
     * -------------------------------------------------------------------- */
    runningState,

    /* ---- Complete ---------------------------------------------------------
     *
     * The completeState signals that the runtime instantiation of the
     * StoryboardElement cannot reach a running state without external
     * interference. If the affected runtime instantiation of the
     * StoryboardElement is defined with a maximumExecutionCount, to be
     * complete implies that there are no more executions left to run, or a
     * stopTransition has occurred.
     *
     * Checking for completeness involves verifying if the given runtime
     * instantiation of the StoryboardElement still has executions left upon
     * finishing the runningState. This check returns false if there are
     * executions left. This check returns true if there are no executions
     * left, or if the maximumExecutionCount is not defined in the
     * StoryboardElement.
     *
     * Resetting the completeState can only be achieved externally by the
     * parent StoryboardElement whose child is in the completeState. This may
     * only occur if the parent initiates a new execution.
     *
     * -------------------------------------------------------------------- */
    completeState,

    /* ---- Start ------------------------------------------------------------
     *
     * The startTransition symbolizes that the execution of the runtime
     * instantiation is now starting. The startTransition can be used in
     * conditions to trigger based on this transition.
     *
     * -------------------------------------------------------------------- */
    startTransition,

    /* ---- End --------------------------------------------------------------
     *
     * The endTransition occurs when the runtime instantiation of the
     * StoryboardElement accomplishes its goal. Once the endTransition occurs,
     * a check for completeness is made. A positive outcome moves the state
     * machine to the completeState, whereas a negative outcome moves the
     * state machine to the standbyState. The endTransition can be used in
     * conditions to trigger based on this transition.
     *
     * -------------------------------------------------------------------- */
    endTransition,

    /* ---- Stop -------------------------------------------------------------
     *
     * The stopTransition marks the reception of a stopTrigger or the
     * storyboard element is overridden (applicable for Event and Action).
     * This implies that the stopTransition cannot be reached other than with
     * an external intervention to the runtime instantiation of the
     * StoryboardElement.
     *
     * When a runtime instantiation of a StoryboardElement goes through a
     * stopTransition, all of its child elements are also forced to go through
     * the same transition. The stopTransition can be used in conditions to
     * trigger based on this transition.
     *
     * -------------------------------------------------------------------- */
    stopTransition,

    /* ---- Skip -------------------------------------------------------------
     *
     * Transition marking the moment an element is asked to move to the
     * runningState but is instead skipped so it remains in the standbyState
     * (only for Event instances). The skipTransition can be used in
     * conditions to trigger based on this transition.
     *
     * -------------------------------------------------------------------- */
    skipTransition,
  } value;

  explicit constexpr StoryboardElementState(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(
  std::basic_istream<Ts...> & is,
  StoryboardElementState & state)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    state.value = StoryboardElementState::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(startTransition);
  BOILERPLATE(endTransition);
  BOILERPLATE(stopTransition);
  BOILERPLATE(skipTransition);
  BOILERPLATE(completeState);
  BOILERPLATE(runningState);
  BOILERPLATE(standbyState);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type StoryboardElementState";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const StoryboardElementState & state)
{
  switch (state) {
    #define BOILERPLATE(ID) case StoryboardElementState::ID: return os << #ID;

    BOILERPLATE(startTransition);
    BOILERPLATE(endTransition);
    BOILERPLATE(stopTransition);
    BOILERPLATE(skipTransition);
    BOILERPLATE(completeState);
    BOILERPLATE(runningState);
    BOILERPLATE(standbyState);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class StoryboardElementState holds unexpected value " <<
        static_cast<StoryboardElementState::value_type>(state.value);
      throw ImplementationFault {ss.str()};
  }
}

static const auto standby_state {
  make<StoryboardElementState>(StoryboardElementState::standbyState)};
static const auto running_state {
  make<StoryboardElementState>(StoryboardElementState::runningState)};
static const auto complete_state {
  make<StoryboardElementState>(StoryboardElementState::completeState)};

static const auto start_transition {make<StoryboardElementState>(
    StoryboardElementState::startTransition)};
static const auto end_transition {make<StoryboardElementState>(
    StoryboardElementState::endTransition)};
static const auto stop_transition {make<StoryboardElementState>(
    StoryboardElementState::stopTransition)};
static const auto skip_transition {make<StoryboardElementState>(
    StoryboardElementState::skipTransition)};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_
