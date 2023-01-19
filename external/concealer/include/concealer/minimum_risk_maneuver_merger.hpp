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

#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#ifdef USE_ADAPI_V1_MSGS
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#endif

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
  auto set(autoware_auto_system_msgs::msg::EmergencyState & msg) -> void
  {
    if (source == MinimumRiskManeuverSource::autoware_adapi_v1_msgs) {
      std::cout
        << "Multiple source of MinimumRiskManeuverState are exist. It must be the only."
           "It is possible that the combination of Autoware-related repositories is incorrect."
        << std::endl;
    } else {
      source = MinimumRiskManeuverSource::autoware_auto_system_msgs;
      state_name = extractStateName<autoware_auto_system_msgs::msg::EmergencyState>(msg);
    }
  }

#ifdef USE_ADAPI_V1_MSGS
  auto set(autoware_adapi_v1_msgs::msg::MrmState & msg) -> void
  {
    if (source == MinimumRiskManeuverSource::autoware_auto_system_msgs) {
      std::cout
        << "Multiple source of MinimumRiskManeuverState are exist. It must be the only."
           " It is possible that the combination of Autoware-related repositories is incorrect."
        << std::endl;
    } else {
      source = MinimumRiskManeuverSource::autoware_adapi_v1_msgs;
      state_name = extractStateName<autoware_adapi_v1_msgs::msg::MrmState>(msg);
      behavior_name = extractBehaviorName<autoware_adapi_v1_msgs::msg::MrmState>(msg);
    }
  }
#endif

  auto getStateName() const -> std::string
  {
    if (source == MinimumRiskManeuverSource::none) {
      throw common::Error(
        "No source for MinimumRiskManeuverState exist."
        "Please check source topic name and its existence");
    } else {
      return state_name;
    }
  }

  auto getBehaviorName() const -> std::string
  {
    switch (source) {
      case MinimumRiskManeuverSource::none:
        throw common::Error(
          "No source for MinimumRiskManeuverState exist."
          "Please check source topic name and its existence");
      case MinimumRiskManeuverSource::autoware_auto_system_msgs:
        throw common::Error(
          "the Autoware uses autoware_auto_system_msgs for EmergencyState, which doesn't support "
          "behavior of MinimumRiskManeuver");
      case MinimumRiskManeuverSource::autoware_adapi_v1_msgs:
        return behavior_name;
    }
  }

private:
  template <typename T>
  auto extractStateName(const T & msg) const -> std::string
  {
    using RAW_T = std::decay_t<T>;
#define CASE(IDENTIFIER) \
  case T::IDENTIFIER:    \
    return #IDENTIFIER;  \
    break

    if constexpr (std::is_same_v<RAW_T, autoware_auto_system_msgs::msg::EmergencyState>) {
      switch (msg.state) {
        CASE(MRM_FAILED);
        CASE(MRM_OPERATING);
        CASE(MRM_SUCCEEDED);
        CASE(NORMAL);
        CASE(OVERRIDE_REQUESTING);

        default:
          throw common::Error(
            "Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
      }
    }
#ifdef USE_ADAPI_V1_MSG
    else if constexpr (std::is_same_v<RAW_T, autoware_adapi_v1_msgs::msg::MrmState>) {
      switch (msg.state) {
        CASE(MRM_FAILED);
        CASE(MRM_OPERATING);
        CASE(MRM_SUCCEEDED);
        CASE(NORMAL);
        CASE(UNKNOWN);

        default:
          throw common::Error(
            "Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
      }
    }
#endif
    else {
      throw common::Error("Unsupported MrmState type : ", typeid(RAW_T).name());
    }
#undef CASE
  }

  template <typename T>
  auto extractBehaviorName(const T & msg) const -> std::string
  {
    using RAW_T = std::decay_t<T>;
#define CASE(IDENTIFIER) \
  case T::IDENTIFIER:    \
    return #IDENTIFIER;  \
    break

    if constexpr (std::is_same_v<RAW_T, autoware_auto_system_msgs::msg::EmergencyState>) {
      throw common::Error(
        "autoware_auto_system_msgs::msg::EmergencyState has no behavior field for "
        "MinimumRiskManeuver");
    }
#ifdef USE_ADAPI_V1_MSG
    else if constexpr (std::is_same_v<RAW_T, autoware_adapi_msgs::msg::MrmState>) {
      switch (msg.behavior) {
        CASE(COMFORTABLE_STOP);
        CASE(EMERGENCY_STOP);
        CASE(NONE);
        CASE(UNKNOWN);

        default:
          throw common::Error(
            "Unsupported MrmState::behavior, number : ", static_cast<int>(msg.behavior));
      }
    }
#endif
    else {
      throw common::Error("Unsupported type of MrmState");
    }
#undef CASE
  }
};
}  // namespace concealer
#endif  //CONCEALER__MINIMUM_RISK_MANEUVER_MERGER_HPP_
