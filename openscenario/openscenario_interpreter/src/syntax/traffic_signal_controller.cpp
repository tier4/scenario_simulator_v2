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
#include <set>
#include <unordered_map>

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

  updatePredictions();

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
  if (shouldChangePhaseToBegin()) {
    return changePhaseTo(std::begin(phases));
  } else if (currentPhaseExceeded()) {
    return changePhaseTo(std::next(current_phase));
  } else {
    return unspecified;
  }
}

auto TrafficSignalController::updatePredictions() -> void
{
  if (current_phase != std::end(phases)) {
    clearV2ITrafficLightsStatePrediction();

    std::unordered_map<lanelet::Id, std::vector<std::tuple<double, std::string>>> predictions_by_id;
    /*
    NOTE:
      This parameter was set with the help of developers familiar with the implementation
      of ROS driver nodes for V2I traffic lights.
      However, the specifications, including this parameter, may change in the future.
   */
    constexpr size_t OUTPUT_PREDICTION_SIZE = 6;

    const auto current_time = evaluateSimulationTime();
    const auto current_phase_elapsed = current_time - current_phase_started_at;
    const double remaining_time_in_current_phase =
      (*current_phase).duration - current_phase_elapsed;
    double accumulated_time = 0.0;

    auto phase_iter = current_phase;
    std::size_t processed_phases_number = 0;

    auto process_phase = [&](const auto & phase, const double phase_time_seconds) -> bool {
      accumulated_time += phase_time_seconds;
      bool is_added = false;
      for (const auto & traffic_signal_state : (*phase).traffic_signal_states) {
        if (
          traffic_signal_state.trafficSignalType() == TrafficSignalState::TrafficSignalType::v2i) {
          is_added = true;
          predictions_by_id[traffic_signal_state.id()].emplace_back(
            accumulated_time, traffic_signal_state.state);
        }
      }
      return is_added;
    };

    if (process_phase(phase_iter, remaining_time_in_current_phase)) {
      ++processed_phases_number;
    }

    while (processed_phases_number < OUTPUT_PREDICTION_SIZE) {
      ++phase_iter;
      if (processed_phases_number == 0 && accumulated_time > cycleTime()) {
        break;
      } else if (process_phase(phase_iter, (*phase_iter).duration)) {
        ++processed_phases_number;
      }
    }

    for (const auto & [lanelet_id, predictions] : predictions_by_id) {
      for (const auto & [time_offset, state] : predictions) {
        setV2ITrafficLightsStatePrediction(lanelet_id, state, time_offset);
      }
    }
  }
}

auto TrafficSignalController::notifyBegin() -> void
{
  change_to_begin_time = evaluateSimulationTime() + delay;
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
