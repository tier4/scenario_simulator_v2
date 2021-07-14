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

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Phases of a TrafficSignalController.
   *
   * ------------------------------------------------------------------------ */
  std::list<Phase> phases;

private:
  CircularIterator<std::list<Phase>> current_phase;

  boost::optional<decltype(getCurrentTime())> change_to_begin_time;

  decltype(getCurrentTime()) current_phase_started_at;

  std::vector<std::shared_ptr<TrafficSignalController>> observers;

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
      THROW_SYNTAX_ERROR(
        "TrafficSignalController (", name, "): delay must not be a negative number");
    }

    if (not std::isnan(delay) and reference.empty()) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController (", name, "): If delay is set, reference is required");
    }

    if (not reference.empty() and std::isnan(delay)) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController ", std::quoted(name), ": If reference is set, delay is required");
    }
  }

  auto changePhaseByName(const std::string & phase_name)
  {
    auto it = std::find_if(phases.begin(), phases.end(), [&phase_name](const Phase & phase) {
      return phase.name == phase_name;
    });

    if (it == phases.end()) {
      THROW_SYNTAX_ERROR(
        std::quoted(phase_name), " is not declared in TrafficSignalContoller ", std::quoted(name));
    }

    return changePhaseTo(it);
  }

  auto changePhaseTo(const std::list<Phase>::iterator & next) -> Element
  {
    const auto current_time = getCurrentTime();

    if (next == phases.begin()) {
      for (auto & observer : observers) {
        observer->notifyBegin();
      }
    }

    current_phase_started_at = current_time;
    current_phase = next;

    return current_phase != phases.end() ? (*current_phase).evaluate() : unspecified;
  }

  auto currentPhaseExceeded() const -> bool
  {
    return current_phase != phases.end() and
           (*current_phase).duration <= (getCurrentTime() - current_phase_started_at);
  }

  auto currentPhaseName() const { return (*current_phase).name; }

  auto currentPhaseSince() const { return current_phase_started_at; }

  auto cycleTime() const
  {
    return std::accumulate(
      std::cbegin(phases), std::cend(phases), 0,
      [](const auto & sum, const auto & phase) { return sum + phase.duration; });
  }

  auto evaluate()
  {
    if (shouldChangePhaseToBegin()) {
      return changePhaseTo(phases.begin());
    } else if (currentPhaseExceeded()) {
      return changePhaseTo(std::next(current_phase));
    } else {
      return unspecified;
    }
  }

  void notifyBegin() { change_to_begin_time = getCurrentTime() + delay; }

  auto shouldChangePhaseToBegin() -> bool
  {
    if (reference.empty()) {
      return current_phase == phases.end();  // if current_phase haven't been initialized
    } else {
      if (change_to_begin_time.has_value() and (change_to_begin_time.value() < getCurrentTime())) {
        change_to_begin_time = boost::none;
        return true;
      } else {
        return false;
      }
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
