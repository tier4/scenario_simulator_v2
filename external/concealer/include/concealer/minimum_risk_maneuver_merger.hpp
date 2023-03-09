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

#ifndef CONCEALER__MINIMUM_RISK_MANEUVER_MERGER_HPP_
#define CONCEALER__MINIMUM_RISK_MANEUVER_MERGER_HPP_

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
struct MinimumRiskManeuverMerger
{
private:
  enum class MinimumRiskManeuverSource {
    none,
    autoware_auto_system_msgs,
    autoware_adapi_v1_msgs,
  } source = MinimumRiskManeuverSource::none;

  std::string state_name;

  std::string behavior_name;

public:
  //  auto set(const autoware_auto_system_msgs::msg::EmergencyState & msg) -> void;

  //  auto set(const autoware_adapi_v1_msgs::msg::MrmState & msg) -> void;

  //  auto getStateName() const -> std::string;

  //  auto getBehaviorName() const -> std::string;

private:
};

template <typename T>
auto extractStateName(const T & msg) -> std::string;

template <typename T>
auto extractBehaviorName(const T & msg) -> std::string;

}  // namespace concealer
#endif  //CONCEALER__MINIMUM_RISK_MANEUVER_MERGER_HPP_
