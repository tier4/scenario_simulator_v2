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

#ifndef CONCEALER__LEGACY_AUTOWARE_STATE_HPP_
#define CONCEALER__LEGACY_AUTOWARE_STATE_HPP_

#if __has_include(<autoware_system_msgs/msg/autoware_state.hpp>)
#include <autoware_system_msgs/msg/autoware_state.hpp>
#endif

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#endif

#if __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#endif

#if __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#endif

namespace concealer
{
struct LegacyAutowareState
{
  enum value_type {
    undefined,
    initializing,
    waiting_for_route,
    planning,
    waiting_for_engage,
    driving,
    arrived_goal,
    finalizing,
  };

  value_type value;

  constexpr LegacyAutowareState(value_type value) : value(value) {}

#if __has_include(<autoware_system_msgs/msg/autoware_state.hpp>)
  explicit LegacyAutowareState(const autoware_system_msgs::msg::AutowareState & autoware_state)
  : value([&]() {
      std::cout << "LegacyAutowareState(OLD): " << autoware_state.state << std::endl;
      switch (autoware_state.state) {
        case autoware_system_msgs::msg::AutowareState::INITIALIZING:
          return initializing;
        case autoware_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE:
          return waiting_for_route;
        case autoware_system_msgs::msg::AutowareState::PLANNING:
          return planning;
        case autoware_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE:
          return waiting_for_engage;
        case autoware_system_msgs::msg::AutowareState::DRIVING:
          return driving;
        case autoware_system_msgs::msg::AutowareState::ARRIVED_GOAL:
          return arrived_goal;
        case autoware_system_msgs::msg::AutowareState::FINALIZING:
          return finalizing;
      }
    }())
  {
  }
#endif

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>) and \
    __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>) and \
    __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
  explicit LegacyAutowareState(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState & localization_state,
    const autoware_adapi_v1_msgs::msg::RouteState & route_state,
    const autoware_adapi_v1_msgs::msg::OperationModeState & operation_mode_state,
    const rclcpp::Time & now)
  : value([&]() {
      std::cout << "LegacyAutowareState(NEW): " << localization_state.state << ", "
                << route_state.state << ", " << operation_mode_state.mode << std::endl;
      /*
         See
         - https://github.com/autowarefoundation/autoware.universe/blob/e60daf7d1c85208eaac083b90c181e224c2ac513/system/autoware_default_adapi/document/autoware-state.md
         - https://github.com/autowarefoundation/autoware_universe/blob/e60daf7d1c85208eaac083b90c181e224c2ac513/system/autoware_default_adapi/src/compatibility/autoware_state.cpp#L80-L141
      */
      switch (localization_state.state) {
        case autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN:
        case autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNINITIALIZED:
        case autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING:
          return initializing;

        case autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED:
          switch (route_state.state) {
            case autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN:
              return initializing;

            case autoware_adapi_v1_msgs::msg::RouteState::ARRIVED:
              if (const auto duration = now - rclcpp::Time(route_state.stamp);
                  duration.seconds() < 2.0) {
                return arrived_goal;
              }
              [[fallthrough]];

            case autoware_adapi_v1_msgs::msg::RouteState::UNSET:
              return waiting_for_route;

            case autoware_adapi_v1_msgs::msg::RouteState::SET:
            case autoware_adapi_v1_msgs::msg::RouteState::CHANGING:
              switch (operation_mode_state.mode) {
                case autoware_adapi_v1_msgs::msg::OperationModeState::UNKNOWN:
                  return initializing;

                case autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS:
                case autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL:
                case autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE:
                  if (operation_mode_state.is_autoware_control_enabled) {
                    return driving;
                  }
                  [[fallthrough]];

                case autoware_adapi_v1_msgs::msg::OperationModeState::STOP:
                  return operation_mode_state.is_autonomous_mode_available ? waiting_for_engage
                                                                           : planning;

                default:
                  return undefined;
              }

            default:
              return undefined;
          }

        default:
          return undefined;
      }
    }())
  {
  }
#endif

  constexpr operator const char *() const
  {
    switch (value) {
      case initializing:
        return "INITIALIZING";
      case waiting_for_route:
        return "WAITING_FOR_ROUTE";
      case planning:
        return "PLANNING";
      case waiting_for_engage:
        return "WAITING_FOR_ENGAGE";
      case driving:
        return "DRIVING";
      case arrived_goal:
        return "ARRIVED_GOAL";
      case finalizing:
        return "FINALIZING";
      default:
        return "";
    }
  }

  operator std::string() const { return operator const char *(); }

  friend auto operator<<(std::ostream & os, const LegacyAutowareState & state) -> std::ostream &
  {
    return os << static_cast<const char *>(state);
  }
};
}  // namespace concealer

#endif  // CONCEALER__LEGACY_AUTOWARE_STATE_HPP_
