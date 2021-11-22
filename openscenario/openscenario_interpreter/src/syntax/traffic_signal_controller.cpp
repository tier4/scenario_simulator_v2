// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
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
  const auto current_time = getCurrentTime();

  if (next == std::begin(phases)) {
    for (auto & observer : observers) {
      observer->notifyBegin();
    }
  }

  current_phase_started_at = current_time;
  current_phase = next;

  return current_phase != std::end(phases) ? (*current_phase).evaluate() : unspecified;
}

auto TrafficSignalController::currentPhaseExceeded() const -> bool
{
  return current_phase != std::end(phases) and
         (*current_phase).duration <= (getCurrentTime() - current_phase_started_at);
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

auto TrafficSignalController::notifyBegin() -> void
{
  change_to_begin_time = getCurrentTime() + delay;
}

auto TrafficSignalController::shouldChangePhaseToBegin() -> bool
{
  if (reference.empty()) {
    return current_phase == std::end(phases);  // if current_phase haven't been initialized
  } else if (
    change_to_begin_time.has_value() and (change_to_begin_time.value() < getCurrentTime())) {
    change_to_begin_time = boost::none;
    return true;
  } else {
    return false;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
