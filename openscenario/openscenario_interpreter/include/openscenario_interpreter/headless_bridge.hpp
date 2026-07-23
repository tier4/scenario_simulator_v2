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

#ifndef OPENSCENARIO_INTERPRETER__HEADLESS_BRIDGE_HPP_
#define OPENSCENARIO_INTERPRETER__HEADLESS_BRIDGE_HPP_

#ifdef SSV2_HEADLESS_EGO

// Exported (non-inline) bridge over SimulatorCore's private static `core`, compiled INSIDE
// libopenscenario_interpreter. The pybind facade (openscenario_python, a separate .so loaded by
// CPython with RTLD_LOCAL) must call these instead of the header-inline SimulatorCore statics:
// the inline statics would bind to the facade's own view of `core`, which is null because the
// Interpreter populates the copy that lives in this library.
//
// Per-entity truth is composed here (in-process, where `core` is valid) and returned in bulk as
// EntityState, so the facade never assembles state from many boundary calls — it only marshals the
// composed struct into Python objects.

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <cstdint>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

namespace openscenario_interpreter
{
namespace headless
{
// One entity's ground truth, composed in-process. `turn_indicator` is populated only for the ego
// (type == 0); empty for other entities.
struct EntityState
{
  std::string name;
  std::uint8_t type;  // 0=EGO 1=VEHICLE 2=PEDESTRIAN 3=MISC_OBJECT
  std::string action;
  std::string turn_indicator;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Accel accel;
  traffic_simulator_msgs::msg::BoundingBox bounding_box;
};

// Composed state for a single entity / all spawned entities (ego + NPC truth for scoring).
auto entityState(const std::string & name) -> EntityState;
auto entityStates() -> std::vector<EntityState>;

auto simulationTime() -> double;

// In-process injection for the (single, "ego"-named) headless ego.
auto setEgoTrajectory(
  const std::string & ego_ref, const rclcpp::Time & stamp,
  const autoware_planning_msgs::msg::Trajectory & trajectory) -> void;
auto setEgoTurnIndicator(
  const std::string & ego_ref,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & command) -> void;

auto conventionalTrafficLightComposedState(std::int64_t lanelet_id) -> std::string;
}  // namespace headless
}  // namespace openscenario_interpreter

#endif  // SSV2_HEADLESS_EGO
#endif  // OPENSCENARIO_INTERPRETER__HEADLESS_BRIDGE_HPP_
