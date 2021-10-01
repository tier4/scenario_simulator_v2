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
#include <openscenario_interpreter/scope.hpp>
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
  // ID of the traffic signal controller in the road network.
  const String name;

  /*
     The delay to the controller in the reference property. A controller
     having a delay to another one means that its first phase virtually starts
     delaytime seconds after the start of the reference's first phase. This
     can be used to define a progressive signal system, but only makes sense,
     if the total times of all connected controllers are the same. If delay is
     set, reference is required. Unit: s; Range: [0..inf[.
  */
  const Double delay;

  /*
     A reference (ID) to the connected controller in the road network. If
     reference is set, a delay is required.
  */
  const String reference;

  // Phases of a TrafficSignalController.
  std::list<Phase> phases;

private:
  CircularIterator<std::list<Phase>> current_phase;

  boost::optional<double> change_to_begin_time;

  double current_phase_started_at;

  std::vector<std::shared_ptr<TrafficSignalController>> observers;

  friend struct TrafficSignalControllerAction;

  friend struct TrafficSignals;

public:
  explicit TrafficSignalController() = delete;

  explicit TrafficSignalController(TrafficSignalController &&) = delete;

  explicit TrafficSignalController(const TrafficSignalController &) = delete;

  template <typename Node>
  explicit TrafficSignalController(const Node & node, Scope & scope)
  : name(readAttribute<String>("name", node, scope)),
    delay(readAttribute<Double>("delay", node, scope, Double::nan())),
    reference(readAttribute<String>("reference", node, scope, "")),
    phases(readElements<Phase, 0>("Phase", node, scope)),
    current_phase(std::begin(phases), std::end(phases), std::end(phases)),
    change_to_begin_time(boost::none),
    current_phase_started_at(std::numeric_limits<double>::min())
  {
    if (delay < 0) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController ", std::quoted(name), ": delay must not be a negative number");
    }

    if (not std::isnan(delay) and reference.empty()) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController ", std::quoted(name), ": If delay is set, reference is required");
    }

    if (not reference.empty() and std::isnan(delay)) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController ", std::quoted(name), ": If reference is set, delay is required");
    }
  }

  auto changePhaseTo(const String &) -> Element;

  auto changePhaseTo(std::list<Phase>::iterator) -> Element;

  auto currentPhaseExceeded() const -> bool;

  auto currentPhaseName() const -> const String &;

  auto currentPhaseSince() const -> double;

  auto cycleTime() const -> double;

  auto evaluate() -> Element;

  auto notifyBegin() -> void;

  auto shouldChangePhaseToBegin() -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
