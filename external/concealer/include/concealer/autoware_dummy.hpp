// Copyright 2015-2022 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__AUTOWARE_DUMMY_HPP_
#define CONCEALER__AUTOWARE_DUMMY_HPP_

#include <concealer/autoware.hpp>

namespace concealer
{
class AutowareDummy : public Autoware
{
  void sendSIGINT() override{};

public:
  auto engage() -> void override {}
  auto initialize(const geometry_msgs::msg::Pose &) -> void override {}
  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override {}
  auto update() -> void override {}
  auto getAcceleration() const -> double override { return 0.0; }
  auto getAutowareStateMessage() const -> std::string override { return "DummyState"; }
  auto getGearSign() const -> double override { return 1.0; }
  auto getSteeringAngle() const -> double override { return 0.0; }
  auto getVehicleCommand() const -> std::tuple<
    autoware_auto_control_msgs::msg::AckermannControlCommand,
    autoware_auto_vehicle_msgs::msg::GearCommand> override
  {
    return std::tuple<
      autoware_auto_control_msgs::msg::AckermannControlCommand,
      autoware_auto_vehicle_msgs::msg::GearCommand>();
  }
  auto getVelocity() const -> double override { return 0.0; }
  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
  auto restrictTargetSpeed(double) const -> double override { return 0.0; }
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_DUMMY_HPP_
