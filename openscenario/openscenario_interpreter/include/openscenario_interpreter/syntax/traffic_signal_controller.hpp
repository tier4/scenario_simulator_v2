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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_

#include <openscenario_interpreter/iterator/circular_iterator.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/phase.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignalController ------------------------------------------------
 *
 *  <xsd:complexType name="TrafficSignalController">
 *    <xsd:sequence>
 *      <xsd:element name="Phase" minOccurs="0" maxOccurs="unbounded" type="Phase"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="delay" type="Double" use="optional"/>
 *    <xsd:attribute name="reference" type="String" use="optional"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalController
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  ID of the traffic signal controller in the road network.
   *
   * ------------------------------------------------------------------------ */
  const String name;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  The delay to the controller in the reference property. A controller
   *  having a delay to another one means that its first phase virtually starts
   *  delaytime seconds after the start of the reference's first phase. This
   *  can be used to define a progressive signal system, but only makes sense,
   *  if the total times of all connected controllers are the same. If delay is
   *  set, reference is required. Unit: s; Range: [0..inf[.
   *
   *  CURRENTLY, IGNORED!!!
   *
   * ------------------------------------------------------------------------ */
  const Double delay;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  A reference (ID) to the connected controller in the road network. If
   *  reference is set, a delay is required.
   *
   *  CURRENTLY, IGNORED!!!
   *
   * ------------------------------------------------------------------------ */
  const String reference;

private:
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Phases of a TrafficSignalController.
   *
   * ------------------------------------------------------------------------ */
  std::list<Phase> phases;

  CircularIterator<std::list<Phase>> current_phase;

  boost::optional<decltype(getCurrentTime())> change_to_begin_time;
  decltype(getCurrentTime()) current_phase_started_at;

  std::vector<std::shared_ptr<TrafficSignalController>> observers;

  auto changePhase(const std::list<Phase>::iterator & next)
  {
    auto current_time = getCurrentTime();

    if (next == phases.begin()) {
      for (auto & observer : observers) {
        observer->notify_begin();
      }
    }

    current_phase_started_at = current_time;
    current_phase = next;

    if (current_phase != phases.end()) {
      return (*current_phase).evaluate();
    } else {
      return unspecified;
    }
  }

  void notify_begin() { change_to_begin_time = getCurrentTime() + delay; }

  friend struct TrafficSignalControllerAction;
  friend struct TrafficSignals;

public:
  explicit TrafficSignalController() = delete;

  explicit TrafficSignalController(TrafficSignalController &&) = delete;

  explicit TrafficSignalController(const TrafficSignalController &) = delete;

  template <typename Node, typename Scope>
  explicit TrafficSignalController(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    delay(
      readAttribute<Double>("delay", node, outer_scope, std::numeric_limits<double>::quiet_NaN())),
    reference(readAttribute<String>("reference", node, outer_scope, String())),
    phases(readElements<Phase, 0>("Phase", node, outer_scope)),
    current_phase(std::begin(phases), std::end(phases), std::end(phases)),
    change_to_begin_time(boost::none),
    current_phase_started_at(std::numeric_limits<decltype(current_phase_started_at)>::min())
  {
    if (delay < 0) {
      THROW_SYNTAX_ERROR("delay must not be a negative number");
    }

    if (delay != 0 && reference.empty()) {
      THROW_SYNTAX_ERROR("If delay is set, reference is required");
    }

    if (!reference.empty() && std::isnan(delay)) {
      THROW_SYNTAX_ERROR("If reference is set, delay is required");
    }
  }

  auto evaluate()
  {
    if (changeToBeginCondition()) {
      return changePhase(phases.begin());
    } else if (theDurationExceeded()) {
      return changePhase(std::next(current_phase));
    } else {
      return unspecified;
    }
  }

  const std::list<Phase> & getPhases() const { return phases; }

  auto cycleTime() const
  {
    Double sum = 0;
    for (auto & each : phases) {
      sum += each.duration;
    }
    return sum;
  }

private:
  auto theDurationExceeded() const -> bool
  {
    if (current_phase != phases.end()) {
      return (*current_phase).duration <= (getCurrentTime() - current_phase_started_at);
    } else {
      return false;
    }
  }

  auto changeToBeginCondition() -> bool
  {
    if (reference.empty()) {
      return current_phase == phases.end();  // if current_phase haven't been initialized
    } else {
      if (change_to_begin_time.has_value() && (change_to_begin_time.value() < getCurrentTime())) {
        change_to_begin_time = boost::none;
        return true;
      } else {
        return false;
      }
    }
  }

  void changePhaseByName(const std::string & phase_name)
  {
    auto it = std::find_if(phases.begin(), phases.end(), [&phase_name](const Phase & phase) {
      return phase.name == phase_name;
    });

    if (it == phases.end()) {
      THROW_SYNTAX_ERROR(phase_name, "is not declared in this TrafficSignalContoller, ", name);
    }

    changePhase(it);
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
