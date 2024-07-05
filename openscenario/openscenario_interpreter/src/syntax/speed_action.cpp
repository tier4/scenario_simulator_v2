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

#include <openscenario_interpreter/functional/equal_to.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/speed_action.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedAction::SpeedAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  speed_action_dynamics(readElement<TransitionDynamics>("SpeedActionDynamics", node, local())),
  speed_action_target(readElement<SpeedActionTarget>("SpeedActionTarget", node, local()))
{
}

auto SpeedAction::accomplished() -> bool
{
  // See OpenSCENARIO 1.1 User Guide Appendix A: Action tables

  auto ends_on_reaching_the_speed = [this]() {
    return speed_action_target.is<AbsoluteTargetSpeed>() or
           not speed_action_target.as<RelativeTargetSpeed>().continuous;
  };

  auto there_is_no_regular_ending = [this]() {
    return speed_action_target.is<RelativeTargetSpeed>() and
           speed_action_target.as<RelativeTargetSpeed>().continuous;
  };

  auto check = [this](auto && actor) {
    if (speed_action_target.is<AbsoluteTargetSpeed>()) {
      return equal_to<double>()(
        speed_action_target.as<AbsoluteTargetSpeed>().value,
        SpeedCondition::evaluate(actor, global().entities));
    } else {
      switch (speed_action_target.as<RelativeTargetSpeed>().speed_target_value_type) {
        case SpeedTargetValueType::delta:
          return equal_to<double>()(
            SpeedCondition::evaluate(
              speed_action_target.as<RelativeTargetSpeed>().entity_ref, global().entities) +
              speed_action_target.as<RelativeTargetSpeed>().value,
            SpeedCondition::evaluate(actor, global().entities));
        case SpeedTargetValueType::factor:
          return equal_to<double>()(
            SpeedCondition::evaluate(
              speed_action_target.as<RelativeTargetSpeed>().entity_ref, global().entities) *
              speed_action_target.as<RelativeTargetSpeed>().value,
            SpeedCondition::evaluate(actor, global().entities));
        default:
          return false;
      }
    }
  };

  if (endsImmediately()) {
    return true;
  } else if (ends_on_reaching_the_speed()) {
    return std::all_of(
      std::begin(accomplishments), std::end(accomplishments), [&](auto && accomplishment) {
        return accomplishment.second = accomplishment.second or check(accomplishment.first);
      });
  } else if (there_is_no_regular_ending()) {
    return false;  // no regular ending
  } else {
    return true;
  }
}

auto SpeedAction::endsImmediately() const -> bool
{
  return speed_action_dynamics.dynamics_shape == DynamicsShape::step;
}

auto SpeedAction::run() -> void {}

auto SpeedAction::start() -> void
{
  accomplishments.clear();

  for (const auto & actor : actors) {
    accomplishments.emplace(actor, false);
  }

  for (auto && each : accomplishments) {
    if (speed_action_target.is<AbsoluteTargetSpeed>()) {
      applySpeedAction(
        std::get<0>(each), speed_action_target.as<AbsoluteTargetSpeed>().value,
        static_cast<traffic_simulator::speed_change::Transition>(
          speed_action_dynamics.dynamics_shape),
        static_cast<traffic_simulator::speed_change::Constraint>(speed_action_dynamics), true);
    } else {
      applySpeedAction(
        std::get<0>(each),
        static_cast<traffic_simulator::speed_change::RelativeTargetSpeed>(
          speed_action_target.as<RelativeTargetSpeed>()),
        static_cast<traffic_simulator::speed_change::Transition>(
          speed_action_dynamics.dynamics_shape),
        static_cast<traffic_simulator::speed_change::Constraint>(speed_action_dynamics),
        speed_action_target.as<RelativeTargetSpeed>().continuous);
    }
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
