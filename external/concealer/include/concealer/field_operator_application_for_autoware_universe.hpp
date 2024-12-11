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
template <>
struct FieldOperatorApplicationFor<AutowareUniverse>
: public FieldOperatorApplication,
  public TransitionAssertion<FieldOperatorApplicationFor<AutowareUniverse>>
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

  auto receiveMrmState(const autoware_adapi_v1_msgs::msg::MrmState & msg) -> void;

  auto receiveEmergencyState(const tier4_external_api_msgs::msg::Emergency & msg) -> void;

  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplicationFor(Ts &&... xs)
  : FieldOperatorApplication(std::forward<decltype(xs)>(xs)...),
    // clang-format off
    getCommand("/control/command/control_cmd", rclcpp::QoS(1), *this),
    getCooperateStatusArray("/api/external/get/rtc_status", rclcpp::QoS(1), *this, [this](const auto & v) { latest_cooperate_status_array = v; }),
    getEmergencyState("/api/external/get/emergency", rclcpp::QoS(1), *this, [this](const auto & v) { receiveEmergencyState(v); }),
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
    getLocalizationState("/api/localization/initialization_state", rclcpp::QoS(1).transient_local(), *this),
#endif
    getMrmState("/api/fail_safe/mrm_state", rclcpp::QoS(1), *this, [this](const auto & v) { receiveMrmState(v); }),
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
