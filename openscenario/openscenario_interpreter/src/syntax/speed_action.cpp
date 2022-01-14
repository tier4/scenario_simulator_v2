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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/speed_action.hpp>

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
  return std::all_of(std::begin(accomplishments), std::end(accomplishments), [](auto && each) {
    return std::get<1>(each);
  });
}

auto SpeedAction::endsImmediately() const -> bool
{
  return speed_action_target.is<AbsoluteTargetSpeed>() and
         speed_action_dynamics.dynamics_shape == DynamicsShape::step;
}

auto SpeedAction::reset() -> void
{
  accomplishments.clear();

  for (const auto & actor : actors) {
    accomplishments.emplace(actor, false);
  }
}

auto SpeedAction::run() -> void
{
  for (auto && each : accomplishments) {
    std::get<1>(each) = std::get<1>(each) or update(EntityRef(std::get<0>(each)));

    // auto && actor = std::get<0>(each);
    //
    // if (speed_action_target.is<AbsoluteTargetSpeed>()) {
    //   connection.requestSpeedChange(
    //     actor,
    //     speed_action_target.as<AbsoluteTargetSpeed>().value,
    //     static_cast<traffic_simulator::SpeedChangeTransition>(
    //       speed_action_dynamics.dynamics_shape),  // NOTE: implicit conversion
    //     traffic_simulator::SpeedChangeConstraint(
    //       static_cast<traffic_simulator::SpeedChangeConstraint::Type>(
    //         speed_action_dynamics.dynamics_dimension),  // NOTE: implicit conversion
    //       speed_action_dynamics.value),
    //     true);
    // } else {
    //   connection.requestSpeedChange(
    //     actor,
    //     traffic_simulator::RelativeTargetSpeed(
    //       speed_action_target.as<RelativeTargetSpeed>().entity_ref,
    //       static_cast<traffic_simulator::RelativeTargetSpeed::Type>(
    //         speed_action_target.as<RelativeTargetSpeed>().speed_target_value_type),  // NOTE: implicit conversion
    //       speed_action_target.as<RelativeTargetSpeed>().value),
    //     static_cast<traffic_simulator::SpeedChangeTransition>(
    //       speed_action_dynamics.dynamics_shape),  // NOTE: implicit conversion
    //     traffic_simulator::SpeedChangeConstraint(
    //       static_cast<traffic_simulator::SpeedChangeConstraint::Type>(
    //         speed_action_dynamics.dynamics_dimension),  // NOTE: implicit conversion
    //       speed_action_dynamics.value),
    //     speed_action_target.as<RelativeTargetSpeed>().continuous);
    // }
  }
}

auto SpeedAction::start() -> void
{
  reset();

  update = [this](const EntityRef & actor)  //
  {
    const auto get_current_absolute_target_speed =
      speed_action_target.getCalculateAbsoluteTargetSpeed();

    switch (speed_action_dynamics.dynamics_shape) {
      case DynamicsShape::step: {
        auto status = getEntityStatus(actor);
        status.action_status.twist.linear.x = get_current_absolute_target_speed();
        setEntityStatus(actor, status);
        setTargetSpeed(actor, status.action_status.twist.linear.x, true);
        break;
      }
      case DynamicsShape::linear:
        setTargetSpeed(actor, get_current_absolute_target_speed(), true);
        break;

      default:
        throw UNSUPPORTED_SETTING_DETECTED(SpeedAction, speed_action_dynamics.dynamics_shape);
    }

    if (speed_action_target.is<RelativeTargetSpeed>()) {
      setTargetSpeed(actor, get_current_absolute_target_speed(), true);
    }

    return speed_action_target.getIsEnd()(actor);
  };
}
}  // namespace syntax
}  // namespace openscenario_interpreter
