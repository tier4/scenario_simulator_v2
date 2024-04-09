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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_

#include <iostream>
#include <openscenario_interpreter/object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- StoryboardElementState -------------------------------------------------
 *
 *  <xsd:simpleType name="StoryboardElementState">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="startTransition"/>
 *          <xsd:enumeration value="endTransition"/>
 *          <xsd:enumeration value="stopTransition"/>
 *          <xsd:enumeration value="skipTransition"/>
 *          <xsd:enumeration value="completeState"/>
 *          <xsd:enumeration value="runningState"/>
 *          <xsd:enumeration value="standbyState"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct StoryboardElementState
{
  enum value_type {

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  This is the default initialization state of a StoryboardElement. When
     *  it is in this state, the runtime instantiation of the StoryboardElement
     *  is ready to execute once given a startTrigger. A runtime instantiation
     *  of any StoryboardElement is created once its parent element is in the
     *  standbyState. From the standbyState, the Story element instantaneously
     *  transitions into the runningState.
     *
     *  Default constructor select this.
     *
     * ---------------------------------------------------------------------- */
    standbyState,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  The runningState symbolizes that the execution of the runtime
     *  instantiation is now ongoing and has not yet accomplished its goal.
     *
     *  The concept of accomplishing a goal varies depending on the type of
     *  StoryboardElement under consideration:
     *
     *  Action
     *    An Action's goal is a function of the Action type and cannot be
     *    generalized. Accomplishing an Action's goal will involve meeting some
     *    arbitrary prerequisites related with the Action type (for example, a
     *    SpeedAction accomplishes its goal when the considered Entity is
     *    travelling at the prescribed speed). If an Action is acting on an
     *    EntitySelection, all instances of Entity within the selection have to
     *    complete in order to reach the completeState of the Action.
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
     * ---------------------------------------------------------------------- */
    runningState,

    /* ---- NOTE ---------------------------------------------------------------
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
     * ---------------------------------------------------------------------- */
    completeState,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  The startTransition symbolizes that the execution of the runtime
     *  instantiation is now starting. The startTransition can be used in
     *  conditions to trigger based on this transition.
     *
     * ---------------------------------------------------------------------- */
    startTransition,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  The endTransition occurs when the runtime instantiation of the
     *  StoryboardElement accomplishes its goal. Once the endTransition occurs,
     *  a check for completeness is made. A positive outcome moves the state
     *  machine to the completeState, whereas a negative outcome moves the
     *  state machine to the standbyState. The endTransition can be used in
     *  conditions to trigger based on this transition.
     *
     * ---------------------------------------------------------------------- */
    endTransition,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  The stopTransition marks the reception of a stopTrigger or the
     *  storyboard element is overridden (applicable for Event and Action).
     *  This implies that the stopTransition cannot be reached other than with
     *  an external intervention to the runtime instantiation of the
     *  StoryboardElement.
     *
     *  When a runtime instantiation of a StoryboardElement goes through a
     *  stopTransition, all of its child elements are also forced to go through
     *  the same transition. The stopTransition can be used in conditions to
     *  trigger based on this transition.
     *
     * ---------------------------------------------------------------------- */
    stopTransition,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  Transition marking the moment an element is asked to move to the
     *  runningState but is instead skipped so it remains in the standbyState
     *  (only for Event instances). The skipTransition can be used in
     *  conditions to trigger based on this transition.
     *
     * ---------------------------------------------------------------------- */
    skipTransition,
  } value;

  StoryboardElementState() = default;

  explicit StoryboardElementState(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, StoryboardElementState &) -> std::istream &;

auto operator<<(std::ostream &, const StoryboardElementState &) -> std::ostream &;

// clang-format off
extern const Object start_transition;
extern const Object   end_transition;
extern const Object  stop_transition;
extern const Object  skip_transition;
extern const Object   complete_state;
extern const Object    running_state;
extern const Object    standby_state;
// clang-format on
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_STATE_HPP_
