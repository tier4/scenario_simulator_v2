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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalController::TrafficSignalController(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  delay(readAttribute<Double>("delay", node, scope, Double::nan())),
  reference(readAttribute<String>("reference", node, scope, "")),
  phases(readElements<Phase, 0>("Phase", node, scope)),
  current_phase(std::begin(phases), std::end(phases), std::end(phases)),
  change_to_begin_time(std::nullopt),
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

auto TrafficSignalController::changePhaseTo(const String & phase_name) -> Object
{
  auto iter = std::find_if(std::begin(phases), std::end(phases), [&](const auto & phase) {
    return phase.name == phase_name;
  });

  if (iter == std::end(phases)) {
    THROW_SYNTAX_ERROR(
      std::quoted(phase_name), " is not declared in TrafficSignalController ", std::quoted(name));
  } else {
    return changePhaseTo(iter);
  }
}

auto TrafficSignalController::changePhaseTo(std::list<Phase>::iterator next) -> Object
{
  if (next == std::begin(phases)) {
    for (auto & observer : observers) {
      observer->notifyBegin();
    }
  }

  current_phase_started_at = evaluateSimulationTime();
  current_phase = next;

  return current_phase != std::end(phases) ? (*current_phase).evaluate() : unspecified;
}

auto TrafficSignalController::currentPhaseExceeded() const -> bool
{
  return current_phase != std::end(phases) and
         (*current_phase).duration <= (evaluateSimulationTime() - current_phase_started_at);
}

auto TrafficSignalController::currentPhaseName() const -> const String &
{
  return (*current_phase).name;
}

auto TrafficSignalController::currentPhaseSince() const -> double
{
  return current_phase_started_at;
}

auto TrafficSignalController::cycleTime() const -> double
{
  return std::accumulate(
    std::cbegin(phases), std::cend(phases), 0,
    [](const auto & sum, const auto & phase) { return sum + phase.duration; });
}

auto TrafficSignalController::evaluate() -> Object
{
  auto updated = [&]() {
    if (shouldChangePhaseToBegin()) {
      return changePhaseTo(std::begin(phases));
    } else if (currentPhaseExceeded()) {
      return changePhaseTo(std::next(current_phase));
    } else {
      return unspecified;
    }
  }();

  auto rest_time_to_red = restTimeToRed();
  double current_phase_rest_time = [this]() {
    if ((*current_phase).duration == Double::infinity()) {
      return -1.0;
    } else {
      return (*current_phase).duration - (evaluateSimulationTime() - current_phase_started_at);
    }
  }();

  for (const auto traffic_signal_state : (*current_phase).traffic_signal_states) {
    setV2ITrafficLightExtraInfo(
      boost::lexical_cast<std::int64_t>(traffic_signal_state.traffic_signal_id),
      current_phase_rest_time, rest_time_to_red);
  }
  return updated;
}

auto TrafficSignalController::notifyBegin() -> void
{
  change_to_begin_time = evaluateSimulationTime() + delay;
}

auto TrafficSignalController::restTimeToRed() const -> double
{
  auto is_red = [](const auto & phase) {
    for (const auto & traffic_signal_state : phase.traffic_signal_states) {
      for (traffic_simulator::TrafficLight & traffic_light :
           getConventionalTrafficLights(traffic_signal_state.id())) {
        for (auto & bulb : traffic_light.bulbs) {
          if (bulb.is(traffic_simulator::TrafficLight::Color::Value::red)) {
            return true;
          }
        }
      }
    }
    return false;
  };

  if (current_phase == std::end(phases)) {
    return 0;
  } else if (is_red(*current_phase)) {
    return 0;
  } else {
    double rest_time_to_red =
      (*current_phase).duration - (evaluateSimulationTime() - current_phase_started_at);
    auto iterator = current_phase;
    auto move_next_phase = [&]() {
      ++iterator;
      return iterator;
    };
    move_next_phase();
    while (not is_red(*iterator)) {
      if (rest_time_to_red > cycleTime()) {
        // no red phase
        return 0;
      }
      rest_time_to_red += (*iterator).duration;
      move_next_phase();
    }
    return rest_time_to_red;
  }
}

auto TrafficSignalController::shouldChangePhaseToBegin() -> bool
{
  if (reference.empty()) {
    return current_phase == std::end(phases);  // if current_phase haven't been initialized
  } else if (
    change_to_begin_time.has_value() and change_to_begin_time.value() < evaluateSimulationTime()) {
    change_to_begin_time = std::nullopt;
    return true;
  } else {
    return false;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
