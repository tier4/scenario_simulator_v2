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

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#endif

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <concealer/autoware_universe.hpp>
#include <concealer/field_operator_application.hpp>
#include <concealer/publisher_wrapper.hpp>
#include <concealer/service_with_validation.hpp>
#include <concealer/task_queue.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/trajectory.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

namespace concealer
{
#define DEFINE_STATIC_DATA_MEMBER_DETECTOR(NAME)                                    \
  template <typename T, typename = void>                                            \
  struct HasStatic##NAME : public std::false_type                                   \
  {                                                                                 \
  };                                                                                \
                                                                                    \
  template <typename T>                                                             \
  struct HasStatic##NAME<T, std::void_t<decltype(T::NAME)>> : public std::true_type \
  {                                                                                 \
  }

DEFINE_STATIC_DATA_MEMBER_DETECTOR(NONE);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(LANE_CHANGE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(LANE_CHANGE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(GOAL_PLANNER);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(START_PLANNER);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(PULL_OUT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(TRAFFIC_LIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(INTERSECTION);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(INTERSECTION_OCCLUSION);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(CROSSWALK);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(BLIND_SPOT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(DETECTION_AREA);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(NO_STOPPING_AREA);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(OCCLUSION_SPOT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_BY_LC_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_BY_LC_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(NO_DRIVABLE_LANE);

// For MrmState::behavior
DEFINE_STATIC_DATA_MEMBER_DETECTOR(COMFORTABLE_STOP);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EMERGENCY_STOP);
// DEFINE_STATIC_DATA_MEMBER_DETECTOR(NONE); // NOTE: This is defined above.
DEFINE_STATIC_DATA_MEMBER_DETECTOR(UNKNOWN);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(PULL_OVER);

#undef DEFINE_STATIC_DATA_MEMBER_DETECTOR

template <>
struct FieldOperatorApplicationFor<AutowareUniverse> : public FieldOperatorApplication
{
  // clang-format off
  SubscriberWrapper<autoware_control_msgs::msg::Control>                          getCommand;
  SubscriberWrapper<tier4_rtc_msgs::msg::CooperateStatusArray>                    getCooperateStatusArray;
  SubscriberWrapper<tier4_external_api_msgs::msg::Emergency>                      getEmergencyState;
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  SubscriberWrapper<autoware_adapi_v1_msgs::msg::LocalizationInitializationState> getLocalizationState;
#endif
  SubscriberWrapper<autoware_adapi_v1_msgs::msg::MrmState>                        getMrmState;
  SubscriberWrapper<tier4_planning_msgs::msg::PathWithLaneId>                     getPathWithLaneId;
  SubscriberWrapper<tier4_planning_msgs::msg::Trajectory>                         getTrajectory;
  SubscriberWrapper<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>            getTurnIndicatorsCommandImpl;

  ServiceWithValidation<autoware_adapi_v1_msgs::srv::ClearRoute>                  requestClearRoute;
  ServiceWithValidation<tier4_rtc_msgs::srv::CooperateCommands>                   requestCooperateCommands;
  ServiceWithValidation<tier4_external_api_msgs::srv::Engage>                     requestEngage;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::InitializeLocalization>      requestInitialPose;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::SetRoutePoints>              requestSetRoutePoints;
  ServiceWithValidation<tier4_rtc_msgs::srv::AutoModeWithModule>                  requestSetRtcAutoMode;
  ServiceWithValidation<tier4_external_api_msgs::srv::SetVelocityLimit>           requestSetVelocityLimit;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::ChangeOperationMode>         requestEnableAutowareControl;
  // clang-format on

  tier4_rtc_msgs::msg::CooperateStatusArray latest_cooperate_status_array;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplicationFor(Ts &&... xs)
  : FieldOperatorApplication(std::forward<decltype(xs)>(xs)...),
    // clang-format off
    getCommand("/control/command/control_cmd", rclcpp::QoS(1), *this),
    getCooperateStatusArray("/api/external/get/rtc_status", rclcpp::QoS(1), *this, [this](const auto & v) { latest_cooperate_status_array = v; }),
    getEmergencyState("/api/external/get/emergency", rclcpp::QoS(1), *this, [this](const auto & message) {
      if (message.emergency) {
        throw common::Error("Emergency state received");
      }
    }),
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
    getLocalizationState("/api/localization/initialization_state", rclcpp::QoS(1).transient_local(), *this),
#endif
    getMrmState("/api/fail_safe/mrm_state", rclcpp::QoS(1), *this, [this](const auto & message) {
      auto state_name_of = [](auto state) constexpr {
        switch (state) {
          case autoware_adapi_v1_msgs::msg::MrmState::MRM_FAILED:
            return "MRM_FAILED";
          case autoware_adapi_v1_msgs::msg::MrmState::MRM_OPERATING:
            return "MRM_OPERATING";
          case autoware_adapi_v1_msgs::msg::MrmState::MRM_SUCCEEDED:
            return "MRM_SUCCEEDED";
          case autoware_adapi_v1_msgs::msg::MrmState::NORMAL:
            return "NORMAL";
          case autoware_adapi_v1_msgs::msg::MrmState::UNKNOWN:
            return "UNKNOWN";
          default:
            throw common::Error(
              "Unexpected autoware_adapi_v1_msgs::msg::MrmState::state, number: ", state);
        }
      };

      auto behavior_name_of = [](auto behavior) constexpr {
        if constexpr (HasStaticCOMFORTABLE_STOP<autoware_adapi_v1_msgs::msg::MrmState>::value) {
          if (behavior == autoware_adapi_v1_msgs::msg::MrmState::COMFORTABLE_STOP) {
            return "COMFORTABLE_STOP";
          }
        }
        if constexpr (HasStaticEMERGENCY_STOP<autoware_adapi_v1_msgs::msg::MrmState>::value) {
          if (behavior == autoware_adapi_v1_msgs::msg::MrmState::EMERGENCY_STOP) {
            return "EMERGENCY_STOP";
          }
        }
        if constexpr (HasStaticNONE<autoware_adapi_v1_msgs::msg::MrmState>::value) {
          if (behavior == autoware_adapi_v1_msgs::msg::MrmState::NONE) {
            return "NONE";
          }
        }
        if constexpr (HasStaticUNKNOWN<autoware_adapi_v1_msgs::msg::MrmState>::value) {
          if (behavior == autoware_adapi_v1_msgs::msg::MrmState::UNKNOWN) {
            return "UNKNOWN";
          }
        }
        if constexpr (HasStaticPULL_OVER<autoware_adapi_v1_msgs::msg::MrmState>::value) {
          if (behavior == autoware_adapi_v1_msgs::msg::MrmState::PULL_OVER) {
            return "PULL_OVER";
          }
        }
        throw common::Error(
          "Unexpected autoware_adapi_v1_msgs::msg::MrmState::behavior, number: ", behavior);
      };

      minimum_risk_maneuver_state = state_name_of(message.state);
      minimum_risk_maneuver_behavior = behavior_name_of(message.behavior);
    }),
    getPathWithLaneId("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", rclcpp::QoS(1), *this),
    getTrajectory("/api/iv_msgs/planning/scenario_planning/trajectory", rclcpp::QoS(1), *this),
    getTurnIndicatorsCommandImpl("/control/command/turn_indicators_cmd", rclcpp::QoS(1), *this),
    requestClearRoute("/api/routing/clear_route", *this),
    requestCooperateCommands("/api/external/set/rtc_commands", *this),
    requestEngage("/api/external/set/engage", *this),
    requestInitialPose("/api/localization/initialize", *this),
    // NOTE: /api/routing/set_route_points takes a long time to return. But the specified duration is not decided by any technical reasons.
    requestSetRoutePoints("/api/routing/set_route_points", *this, std::chrono::seconds(10)),
    requestSetRtcAutoMode("/api/external/set/rtc_auto_mode", *this),
    requestSetVelocityLimit("/api/autoware/set/velocity_limit", *this),
    requestEnableAutowareControl("/api/operation_mode/enable_autoware_control", *this)
  // clang-format on
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
