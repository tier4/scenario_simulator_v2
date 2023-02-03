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

#include <concealer/minimum_risk_maneuver_merger.hpp>

namespace concealer
{
//auto MinimumRiskManeuverMerger::set(const autoware_auto_system_msgs::msg::EmergencyState & msg)
//  -> void
//{
//  if (source == MinimumRiskManeuverSource::autoware_adapi_v1_msgs) {
//    std::cout
//      << "Multiple source of MinimumRiskManeuverState are exist. It must be the only."
//         "It is possible that the combination of Autoware-related repositories is incorrect."
//      << std::endl;
//  } else {
//    std::cout << "Input from auto msg" << std::endl;
//    source = MinimumRiskManeuverSource::autoware_auto_system_msgs;
//    state_name = extractStateName<autoware_auto_system_msgs::msg::EmergencyState>(msg);
//  }
//}
//
//#ifdef USE_ADAPI_V1_MSGS
//auto MinimumRiskManeuverMerger::set(const autoware_adapi_v1_msgs::msg::MrmState & msg) -> void
//{
//  if (source == MinimumRiskManeuverSource::autoware_auto_system_msgs) {
//    std::cout
//      << "Multiple source of MinimumRiskManeuverState are exist. It must be the only."
//         " It is possible that the combination of Autoware-related repositories is incorrect."
//      << std::endl;
//  } else {
//    source = MinimumRiskManeuverSource::autoware_adapi_v1_msgs;
//    state_name = extractStateName<autoware_adapi_v1_msgs::msg::MrmState>(msg);
//    behavior_name = extractBehaviorName<autoware_adapi_v1_msgs::msg::MrmState>(msg);
//  }
//}
//#endif  // USE_ADAPI_V1_MSGS

//auto MinimumRiskManeuverMerger::getStateName() const -> std::string
//{
//  if (source == MinimumRiskManeuverSource::none) {
//    std::cout << "No source for MinimumRiskManeuverState exist."
//                 "Please check source topic name and its existence"
//              << std::endl;
//    return "";
//  } else {
//    std::cout << "Output State : " << state_name << std::endl;
//    return state_name;
//  }
//}
//
//auto MinimumRiskManeuverMerger::getBehaviorName() const -> std::string
//{
//  switch (source) {
//    case MinimumRiskManeuverSource::none:
//      std::cout << "No source for MinimumRiskManeuverState exist."
//                   "Please check source topic name and its existence"
//                << std::endl;
//      return "";
//    case MinimumRiskManeuverSource::autoware_auto_system_msgs:
//      throw common::Error(
//        "the Autoware uses autoware_auto_system_msgs for EmergencyState, which doesn't support "
//        "behavior of MinimumRiskManeuver");
//    case MinimumRiskManeuverSource::autoware_adapi_v1_msgs:
//      return behavior_name;
//  }
//}

template <typename T>
auto extractStateName(const T & msg) -> std::string
{
#define CASE(IDENTIFIER) \
  case T::IDENTIFIER:    \
    return #IDENTIFIER;  \
    break

  if constexpr (std::is_same_v<T, autoware_auto_system_msgs::msg::EmergencyState>) {
    switch (msg.state) {
      CASE(MRM_FAILED);
      CASE(MRM_OPERATING);
      CASE(MRM_SUCCEEDED);
      CASE(NORMAL);
      CASE(OVERRIDE_REQUESTING);

      default:
        throw common::Error("Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
    }
  }
#if __has_include(<autoware_adapi_v1_msgs/msg/mrm_state.hpp>)
  else if constexpr (std::is_same_v<T, autoware_adapi_v1_msgs::msg::MrmState>) {
    switch (msg.state) {
      CASE(MRM_FAILED);
      CASE(MRM_OPERATING);
      CASE(MRM_SUCCEEDED);
      CASE(NORMAL);
      CASE(UNKNOWN);

      default:
        throw common::Error("Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
    }
  }
#endif
  else {
    throw common::Error("Unsupported MrmState type : ", typeid(T).name());
  }
#undef CASE
}

template auto
extractStateName<autoware_auto_system_msgs::msg::EmergencyState>(
  const autoware_auto_system_msgs::msg::EmergencyState &) -> std::string;
#if __has_include(<autoware_adapi_v1_msgs/msg/mrm_state.hpp>)
template auto extractStateName<autoware_adapi_v1_msgs::msg::MrmState>(
  const autoware_adapi_v1_msgs::msg::MrmState &) -> std::string;
#endif

template <typename T>
auto extractBehaviorName(const T & msg) -> std::string
{
#define CASE(IDENTIFIER) \
  case T::IDENTIFIER:    \
    return #IDENTIFIER;  \
    break

  if constexpr (std::is_same_v<T, autoware_auto_system_msgs::msg::EmergencyState>) {
    throw common::Error(
      "autoware_auto_system_msgs::msg::EmergencyState has no behavior field for "
      "MinimumRiskManeuver");
  }
#if __has_include(<autoware_adapi_v1_msgs/msg/mrm_state.hpp>)
  else if constexpr (std::is_same_v<T, autoware_adapi_v1_msgs::msg::MrmState>) {
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

template auto extractBehaviorName<autoware_auto_system_msgs::msg::EmergencyState>(
  const autoware_auto_system_msgs::msg::EmergencyState &) -> std::string;
#if __has_include(<autoware_adapi_v1_msgs/msg/mrm_state.hpp>)
template auto extractBehaviorName<autoware_adapi_v1_msgs::msg::MrmState>(
  const autoware_adapi_v1_msgs::msg::MrmState &) -> std::string;
#endif

}  // namespace concealer
