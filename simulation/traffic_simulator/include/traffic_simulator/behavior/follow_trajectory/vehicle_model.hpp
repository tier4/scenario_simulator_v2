// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_

#include <optional>
#include <traffic_simulator/behavior/follow_trajectory/utils.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
class Vehicle
{
public:
  Vehicle() = default;
  explicit Vehicle(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter)
  : status{entity_status},
    behavior_parameter{behavior_parameter},
    vehicle_parameters{std::nullopt} {};
  Vehicle(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters)
  : status{entity_status},
    behavior_parameter{behavior_parameter},
    vehicle_parameters{vehicle_parameters} {};

  auto getCurrentPosition() const -> geometry_msgs::msg::Point;
  auto getFrontPosition() const -> geometry_msgs::msg::Point;
  auto getCurrentSpeed() const -> double;
  auto getCurrentAcceleration() const -> double;
  auto getAccelerationLimits(double step_time) const -> std::tuple<double, double>;
  auto getName() const -> std::string;
  auto getTime() const -> double;
  auto getMaxSteering() const -> double;
  auto getWheelBase() const -> double;
  auto getOrientation() const -> geometry_msgs::msg::Vector3;

  auto createUpdatedStatus(double steering, double speed, double step_time) const
    -> traffic_simulator_msgs::msg::EntityStatus;
  auto createUpdatedStatus(const geometry_msgs::msg::Vector3 & velocity, double step_time) const
    -> traffic_simulator_msgs::msg::EntityStatus;

private:
  const traffic_simulator_msgs::msg::EntityStatus status;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  const std::optional<const traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_
