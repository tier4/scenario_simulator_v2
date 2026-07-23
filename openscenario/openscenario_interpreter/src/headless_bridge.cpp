// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifdef SSV2_HEADLESS_EGO

#include <openscenario_interpreter/headless_bridge.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

namespace openscenario_interpreter
{
namespace headless
{
namespace
{
// The SimulatorCore passthroughs live as protected statics on NonStandardOperation, reachable only
// by derivation (the same seam the syntax elements use). This helper, compiled into
// libopenscenario_interpreter, gives the composing functions below access to the library's
// populated `core`.
struct Bridge : private SimulatorCore::NonStandardOperation, private SimulatorCore::ConditionEvaluation
{
  using SimulatorCore::NonStandardOperation::getConventionalTrafficLightsComposedState;
  using SimulatorCore::NonStandardOperation::getEgoEntityRef;
  using SimulatorCore::NonStandardOperation::getEntityNamesHeadless;
  using SimulatorCore::NonStandardOperation::getEntityRef;
  // Reuse the existing scenario-time accessor (active()-guarded; returns NaN when inactive).
  using SimulatorCore::ConditionEvaluation::evaluateSimulationTime;
};

auto compose(const std::string & name) -> EntityState
{
  // One entity lookup; every field is read off this reference.
  const auto & entity = Bridge::getEntityRef(name);
  EntityState state;
  state.name = name;
  state.type = entity.getEntityType().type;
  state.action = entity.getCurrentAction();
  state.pose = entity.getMapPose();
  state.twist = entity.getCurrentTwist();
  state.accel = entity.getCurrentAccel();
  state.bounding_box = entity.getBoundingBox();
  if (state.type == traffic_simulator_msgs::msg::EntityType::EGO) {
    state.turn_indicator = Bridge::getEgoEntityRef(name).getTurnIndicatorsCommandName();
  }
  return state;
}
}  // namespace

auto entityState(const std::string & name) -> EntityState { return compose(name); }

auto entityStates() -> std::vector<EntityState>
{
  std::vector<EntityState> states;
  for (const auto & name : Bridge::getEntityNamesHeadless()) {
    states.push_back(compose(name));
  }
  return states;
}

auto simulationTime() -> double { return Bridge::evaluateSimulationTime(); }

auto setEgoTrajectory(
  const std::string & ego_ref, const rclcpp::Time & stamp,
  const autoware_planning_msgs::msg::Trajectory & trajectory) -> void
{
  Bridge::getEgoEntityRef(ego_ref).setDiffusionTrajectory(stamp, trajectory);
}

auto setEgoTurnIndicator(
  const std::string & ego_ref,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & command) -> void
{
  Bridge::getEgoEntityRef(ego_ref).setTurnIndicators(command);
}

auto conventionalTrafficLightComposedState(std::int64_t lanelet_id) -> std::string
{
  return Bridge::getConventionalTrafficLightsComposedState(static_cast<lanelet::Id>(lanelet_id));
}
}  // namespace headless
}  // namespace openscenario_interpreter

#endif  // SSV2_HEADLESS_EGO
