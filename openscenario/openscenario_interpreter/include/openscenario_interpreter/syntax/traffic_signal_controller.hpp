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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_

#include <openscenario_interpreter/iterator/circular_iterator.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/phase.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <optional>
#include <pugixml.hpp>

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
struct TrafficSignalController : private SimulatorCore::ConditionEvaluation,
                                 private SimulatorCore::NonStandardOperation
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

  std::optional<double> change_to_begin_time;

  double current_phase_started_at;

  std::vector<std::shared_ptr<TrafficSignalController>> observers;

  friend struct TrafficSignalControllerAction;

  friend struct TrafficSignals;

public:
  explicit TrafficSignalController() = delete;

  explicit TrafficSignalController(TrafficSignalController &&) = delete;

  explicit TrafficSignalController(const TrafficSignalController &) = delete;

  explicit TrafficSignalController(const pugi::xml_node &, Scope &);

  auto changePhaseTo(const String &) -> Object;

  auto changePhaseTo(std::list<Phase>::iterator) -> Object;

  auto currentPhaseExceeded() const -> bool;

  auto currentPhaseName() const -> const String &;

  auto currentPhaseSince() const -> double;

  auto cycleTime() const -> double;

  auto evaluate() -> Object;

  auto notifyBegin() -> void;

  auto shouldChangePhaseToBegin() -> bool;

  auto generatePredictions(double prediction_horizon_seconds = 30.0) const
    -> std::vector<std::tuple<double, std::string>>;

  auto updatePredictions() -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_HPP_
