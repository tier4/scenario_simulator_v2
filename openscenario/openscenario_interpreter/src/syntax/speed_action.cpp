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
  // return std::all_of(std::begin(accomplishments), std::end(accomplishments), [](auto && each) {
  //   return std::get<1>(each);
  // });
  return true;  // NOTE: dummy
}

auto SpeedAction::endsImmediately() const -> bool
{
  return speed_action_target.is<AbsoluteTargetSpeed>() and
         speed_action_dynamics.dynamics_shape == DynamicsShape::step;
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
      connection.requestSpeedChange(
        std::get<0>(each), speed_action_target.as<AbsoluteTargetSpeed>().value,
        static_cast<traffic_simulator::SpeedChangeTransition>(
          speed_action_dynamics.dynamics_shape),  // NOTE: implicit conversion
        traffic_simulator::SpeedChangeConstraint(
          static_cast<traffic_simulator::SpeedChangeConstraint::Type>(
            speed_action_dynamics.dynamics_dimension),  // NOTE: implicit conversion
          speed_action_dynamics.value),
        true);
    } else {
      connection.requestSpeedChange(
        std::get<0>(each),
        traffic_simulator::RelativeTargetSpeed(
          speed_action_target.as<RelativeTargetSpeed>().entity_ref,
          static_cast<traffic_simulator::RelativeTargetSpeed::Type>(
            speed_action_target.as<RelativeTargetSpeed>()
              .speed_target_value_type),  // NOTE: implicit conversion
          speed_action_target.as<RelativeTargetSpeed>().value),
        static_cast<traffic_simulator::SpeedChangeTransition>(
          speed_action_dynamics.dynamics_shape),  // NOTE: implicit conversion
        traffic_simulator::SpeedChangeConstraint(
          static_cast<traffic_simulator::SpeedChangeConstraint::Type>(
            speed_action_dynamics.dynamics_dimension),  // NOTE: implicit conversion
          speed_action_dynamics.value),
        speed_action_target.as<RelativeTargetSpeed>().continuous);
    }
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
