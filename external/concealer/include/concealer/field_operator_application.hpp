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

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#endif

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <concealer/autoware_universe.hpp>
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
#include <tier4_planning_msgs/msg/trajectory.hpp>
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

  std::string autoware_state = "LAUNCHING";

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  // clang-format off
  using AutowareState                   = autoware_system_msgs::msg::AutowareState;
  using Control                         = autoware_control_msgs::msg::Control;
  using CooperateStatusArray            = tier4_rtc_msgs::msg::CooperateStatusArray;
  using Emergency                       = tier4_external_api_msgs::msg::Emergency;
  using LocalizationInitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using MrmState                        = autoware_adapi_v1_msgs::msg::MrmState;
  using Trajectory                      = tier4_planning_msgs::msg::Trajectory;
  using TurnIndicatorsCommand           = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

  using ClearRoute                      = autoware_adapi_v1_msgs::srv::ClearRoute;
  using CooperateCommands               = tier4_rtc_msgs::srv::CooperateCommands;
  using Engage                          = tier4_external_api_msgs::srv::Engage;
  using InitializeLocalization          = autoware_adapi_v1_msgs::srv::InitializeLocalization;
  using SetRoutePoints                  = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  using AutoModeWithModule              = tier4_rtc_msgs::srv::AutoModeWithModule;
  using SetVelocityLimit                = tier4_external_api_msgs::srv::SetVelocityLimit;
  using ChangeOperationMode             = autoware_adapi_v1_msgs::srv::ChangeOperationMode;

  Subscriber<AutowareState>                   getAutowareState;
  Subscriber<Control>                         getCommand;
  Subscriber<CooperateStatusArray>            getCooperateStatusArray;
  Subscriber<Emergency>                       getEmergencyState;
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  Subscriber<LocalizationInitializationState> getLocalizationState;
#endif
  Subscriber<MrmState>                 getMrmState;
  Subscriber<priority::PathWithLaneId> getPathWithLaneId;
  Subscriber<Trajectory>               getTrajectory;
  Subscriber<TurnIndicatorsCommand>    getTurnIndicatorsCommand;

  Service<ClearRoute>             requestClearRoute;
  Service<CooperateCommands>      requestCooperateCommands;
  Service<Engage>                 requestEngage;
  Service<InitializeLocalization> requestInitialPose;
  Service<SetRoutePoints>         requestSetRoutePoints;
  Service<AutoModeWithModule>     requestSetRtcAutoMode;
  Service<SetVelocityLimit>       requestSetVelocityLimit;
  Service<ChangeOperationMode>    requestEnableAutowareControl;
  // clang-format on

  /*
     The task queue must be deconstructed before any services, so it must be
     the last class data member. (Class data members are constructed in
     declaration order and deconstructed in reverse order.)
  */
  TaskQueue task_queue;

  template <typename Thunk = void (*)()>
  auto waitForAutowareStateToBe(
    const std::string & state, Thunk thunk = [] {})
  {
    thunk();

    while (not finalized.load() and autoware_state != state) {
      if (time_limit <= std::chrono::steady_clock::now()) {
        throw common::AutowareError(
          "Simulator waited for the Autoware state to transition to ", state,
          ", but time is up. The current Autoware state is ",
          (autoware_state.empty() ? "not published yet" : autoware_state));
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

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void;

  auto clearRoute() -> void;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray;

  auto requestAutoModeForCooperation(const std::string &, bool) -> void;

  auto sendCooperateCommand(const std::string &, const std::string &) -> void;

  auto setVelocityLimit(double) -> void;

  auto enableAutowareControl() -> void;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_USER_HPP_
