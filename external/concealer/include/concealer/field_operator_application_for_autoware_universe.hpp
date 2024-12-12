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

#ifndef CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_

#include <concealer/field_operator_application.hpp>

namespace concealer
{
template <>
struct FieldOperatorApplicationFor<AutowareUniverse> : public FieldOperatorApplication
{
  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplicationFor(Ts &&... xs)
  : FieldOperatorApplication(std::forward<decltype(xs)>(xs)...)
  {
  }

  auto engage() -> void override;

  auto engageable() const -> bool override;

  auto engaged() const -> bool override;

  auto getAutowareStateName() const -> std::string override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto getTurnIndicatorsCommand() const
    -> autoware_vehicle_msgs::msg::TurnIndicatorsCommand override;

  auto getEmergencyStateName() const -> std::string override;

  auto getMinimumRiskManeuverBehaviorName() const -> std::string override;

  auto getMinimumRiskManeuverStateName() const -> std::string override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto clearRoute() -> void override;

  auto requestAutoModeForCooperation(const std::string &, bool) -> void override;

  auto sendCooperateCommand(const std::string &, const std::string &) -> void override;

  auto setVelocityLimit(double) -> void override;

  auto enableAutowareControl() -> void override;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_
