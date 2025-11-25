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

#ifndef CONCEALER__AUTOWARE_USER_HPP_
#define CONCEALER__AUTOWARE_USER_HPP_

#include <sys/wait.h>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <concealer/autoware_universe.hpp>
#include <concealer/legacy_autoware_state.hpp>
#include <concealer/path_with_lane_id.hpp>
#include <concealer/publisher.hpp>
#include <concealer/service.hpp>
#include <concealer/subscriber.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <utility>

namespace concealer
{
struct FieldOperatorApplication : public rclcpp::Node
{
  pid_t process_id;

  bool initialized = false;

  std::atomic<bool> finalized = false;

  std::chrono::steady_clock::time_point time_limit;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  // clang-format off
  using AutowareState                   = autoware_system_msgs::msg::AutowareState;
  using Control                         = autoware_control_msgs::msg::Control;
  using CooperateStatusArray            = tier4_rtc_msgs::msg::CooperateStatusArray;
  using Emergency                       = tier4_external_api_msgs::msg::Emergency;
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  using LocalizationInitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
#endif
  using MrmState                        = autoware_adapi_v1_msgs::msg::MrmState;
#if __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
  using OperationModeState              = autoware_adapi_v1_msgs::msg::OperationModeState;
#endif
#if __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>)
  using RouteState                      = autoware_adapi_v1_msgs::msg::RouteState;
#endif
  using TurnIndicatorsCommand           = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

  using ClearRoute                      = autoware_adapi_v1_msgs::srv::ClearRoute;
  using CooperateCommands               = tier4_rtc_msgs::srv::CooperateCommands;
  using Engage                          = tier4_external_api_msgs::srv::Engage;
  using InitializeLocalization          = autoware_adapi_v1_msgs::srv::InitializeLocalization;
  using SetRoute                        = autoware_adapi_v1_msgs::srv::SetRoute;
  using SetRoutePoints                  = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using AutoModeWithModule              = tier4_rtc_msgs::srv::AutoModeWithModule;
  using SetVelocityLimit                = tier4_external_api_msgs::srv::SetVelocityLimit;
  using ChangeOperationMode             = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using Route                           = autoware_adapi_v1_msgs::msg::Route;

  Subscriber<AutowareState>                   getAutowareState;
  Subscriber<Control>                         getCommand;
  Subscriber<CooperateStatusArray>            getCooperateStatusArray;
  Subscriber<Emergency>                       getEmergencyState;
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  Subscriber<LocalizationInitializationState> getLocalizationState;
#endif
  Subscriber<MrmState>                        getMrmState;
#if __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
  Subscriber<OperationModeState>              getOperationModeState;
#endif
  Subscriber<Route>        getRoute;
#if __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>)
  Subscriber<RouteState>                      getRouteState;
#endif
  Subscriber<TurnIndicatorsCommand>           getTurnIndicatorsCommand;

  Service<ClearRoute>             requestClearRoute;
  Service<CooperateCommands>      requestCooperateCommands;
  Service<Engage>                 requestEngage;
  Service<InitializeLocalization> requestInitialPose;
  Service<SetRoute>               requestSetRoute;
  Service<SetRoutePoints>         requestSetRoutePoints;
  Service<AutoModeWithModule>     requestSetRtcAutoMode;
  Service<SetVelocityLimit>       requestSetVelocityLimit;
  Service<ChangeOperationMode>    requestEnableAutowareControl;
  Service<ChangeOperationMode>    requestChangeToStop;
  // clang-format on

  rclcpp::executors::SingleThreadedExecutor executor;

  /*
     The task queue must be deconstructed before any services, so it must be
     the last class data member. (Class data members are constructed in
     declaration order and deconstructed in reverse order.)
  */
  TaskQueue task_queue;

  template <typename Thunk = void (*)()>
  auto waitForAutowareStateToBe(
    const LegacyAutowareState & from_state, const LegacyAutowareState & to_state,
    Thunk thunk = [] {})
  {
    thunk();

    auto not_to_be = [&](auto current_state) {
      return from_state.value <= current_state.value and current_state.value < to_state.value;
    };

    while (not finalized.load() and not_to_be(getLegacyAutowareState())) {
      if (time_limit <= std::chrono::steady_clock::now()) {
        throw common::AutowareError(
          "Simulator waited for the Autoware state to transition to ", to_state,
          ", but time is up. The current Autoware state is ", getLegacyAutowareState());
      } else {
        thunk();
        rclcpp::GenericRate<std::chrono::steady_clock>(std::chrono::seconds(1)).sleep();
      }
    }
  }

  CONCEALER_PUBLIC explicit FieldOperatorApplication(const pid_t);

  ~FieldOperatorApplication();

  auto spinSome() -> void;

  auto engage() -> void;

  auto engageable() const -> bool;

  auto engaged() const -> bool;

  auto initialize(const geometry_msgs::msg::Pose &) -> void;

  [[deprecated(
    "This function was deprecated since version 16.5.0 (released on 20250603). It will be deleted "
    "after a half-year transition period (~20251203). Please use other overloads instead.")]] auto
  plan(const std::vector<geometry_msgs::msg::PoseStamped> &, const bool) -> void;

#if __has_include(<autoware_adapi_v1_msgs/msg/route_option.hpp>)
  using RouteOption = autoware_adapi_v1_msgs::msg::RouteOption;
#else
  using RouteOption = void;
#endif

  auto plan(
    const geometry_msgs::msg::Pose & goal, const std::vector<geometry_msgs::msg::Pose> &,
    const RouteOption &) -> void;

  auto plan(
    const geometry_msgs::msg::Pose & goal,
    const std::vector<autoware_adapi_v1_msgs::msg::RouteSegment> &, const RouteOption &) -> void;

  auto clearRoute() -> void;

  auto getLegacyAutowareState() const -> LegacyAutowareState;

  auto requestAutoModeForCooperation(const std::string &, bool) -> void;

  auto sendCooperateCommand(const std::string &, const std::string &) -> void;

  auto setVelocityLimit(double) -> void;

  auto enableAutowareControl() -> void;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_USER_HPP_
