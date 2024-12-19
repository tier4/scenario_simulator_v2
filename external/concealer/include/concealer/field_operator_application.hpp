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
#include <concealer/launch.hpp>
#include <concealer/publisher.hpp>
#include <concealer/service_with_validation.hpp>
#include <concealer/subscriber.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/trajectory.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <utility>

namespace concealer
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  The magic class 'FieldOperatorApplication' is a class that makes it easy to work with
 *  Autoware from C++. The main features of this class are as follows
 *
 *    (1) Launch Autoware in an independent process upon instantiation of the
 *        class.
 *    (2) Properly terminates the Autoware process started by the constructor
 *        upon destruction of the class.
 *    (3) Probably the simplest instructions to Autoware, consisting of
 *        initialize, plan, and engage.
 *
 * -------------------------------------------------------------------------- */
struct FieldOperatorApplication : public rclcpp::Node,
                                  public TransitionAssertion<FieldOperatorApplication>
{
  std::atomic<bool> is_stop_requested = false;

  bool is_autoware_exited = false;

  const pid_t process_id = 0;

  bool initialize_was_called = false;

  std::string autoware_state;

  tier4_rtc_msgs::msg::CooperateStatusArray latest_cooperate_status_array;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  // clang-format off
  using AutowareState                   = autoware_system_msgs::msg::AutowareState;
  using Control                         = autoware_control_msgs::msg::Control;
  using CooperateStatusArray            = tier4_rtc_msgs::msg::CooperateStatusArray;
  using Emergency                       = tier4_external_api_msgs::msg::Emergency;
  using LocalizationInitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using MrmState                        = autoware_adapi_v1_msgs::msg::MrmState;
  using PathWithLaneId                  = tier4_planning_msgs::msg::PathWithLaneId;
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
  Subscriber<MrmState>                        getMrmState;
  Subscriber<PathWithLaneId>                  getPathWithLaneId;
  Subscriber<Trajectory>                      getTrajectory;
  Subscriber<TurnIndicatorsCommand>           getTurnIndicatorsCommand;

  ServiceWithValidation<ClearRoute>                  requestClearRoute;
  ServiceWithValidation<CooperateCommands>           requestCooperateCommands;
  ServiceWithValidation<Engage>                      requestEngage;
  ServiceWithValidation<InitializeLocalization>      requestInitialPose;
  ServiceWithValidation<SetRoutePoints>              requestSetRoutePoints;
  ServiceWithValidation<AutoModeWithModule>          requestSetRtcAutoMode;
  ServiceWithValidation<SetVelocityLimit>            requestSetVelocityLimit;
  ServiceWithValidation<ChangeOperationMode>         requestEnableAutowareControl;
  // clang-format on

  /*
     The task queue must be deconstructed before any services, so it must be
     the last class data member. (Class data members are constructed in
     declaration order and deconstructed in reverse order.)
  */
  TaskQueue task_queue;

  CONCEALER_PUBLIC explicit FieldOperatorApplication(const pid_t = 0);

  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplication(Ts &&... xs)
  : FieldOperatorApplication(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  ~FieldOperatorApplication();

  auto spinSome() -> void;

  auto engage() -> void;

  auto engageable() const -> bool;

  auto engaged() const -> bool;

  auto initialize(const geometry_msgs::msg::Pose &) -> void;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void;

  auto clearRoute() -> void;

  auto getAutowareStateName() const { return autoware_state; }

  auto getMinimumRiskManeuverBehaviorName() const { return minimum_risk_maneuver_behavior; }

  auto getMinimumRiskManeuverStateName() const { return minimum_risk_maneuver_state; }

  auto getEmergencyStateName() const { return minimum_risk_maneuver_state; }

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray;

  auto initialized() const noexcept { return initialize_was_called; }

  auto requestAutoModeForCooperation(const std::string &, bool) -> void;

  auto rethrow() const { task_queue.rethrow(); }

  auto sendCooperateCommand(const std::string &, const std::string &) -> void;

  auto setVelocityLimit(double) -> void;

  auto enableAutowareControl() -> void;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_USER_HPP_
