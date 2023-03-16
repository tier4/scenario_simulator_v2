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

#ifndef CONCEALER__AUTOWARE_UNIVERSE_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_HPP_

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <concealer/autoware.hpp>
#include <concealer/cooperator.hpp>
#include <concealer/dirty_hack.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/utility/service_with_validation.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

namespace concealer
{
class AutowareUniverse : public Autoware, public TransitionAssertion<AutowareUniverse>
{
  friend class TransitionAssertion<AutowareUniverse>;

  using Acceleration = geometry_msgs::msg::AccelWithCovarianceStamped;
  using Checkpoint = geometry_msgs::msg::PoseStamped;
  using ControlModeReport = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  using GoalPose = geometry_msgs::msg::PoseStamped;
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using TurnIndicatorsReport = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

  CONCEALER_DEFINE_PUBLISHER(Acceleration);
  CONCEALER_DEFINE_PUBLISHER(Checkpoint);
  CONCEALER_DEFINE_PUBLISHER(ControlModeReport);
  CONCEALER_DEFINE_PUBLISHER(GearReport);
  CONCEALER_DEFINE_PUBLISHER(GoalPose);
  CONCEALER_DEFINE_PUBLISHER(InitialPose);
  CONCEALER_DEFINE_PUBLISHER(Odometry);
  CONCEALER_DEFINE_PUBLISHER(SteeringReport);
  CONCEALER_DEFINE_PUBLISHER(TurnIndicatorsReport);
  CONCEALER_DEFINE_PUBLISHER(VelocityReport);

  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;
  using EmergencyState = autoware_auto_system_msgs::msg::EmergencyState;
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
  using MrmState = autoware_adapi_v1_msgs::msg::MrmState;
  using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  CONCEALER_DEFINE_SUBSCRIPTION(AckermannControlCommand);
  CONCEALER_DEFINE_SUBSCRIPTION(AutowareState);
  CONCEALER_DEFINE_SUBSCRIPTION(CooperateStatusArray);
  CONCEALER_DEFINE_SUBSCRIPTION(EmergencyState);
  CONCEALER_DEFINE_SUBSCRIPTION(GearCommand, override);
  CONCEALER_DEFINE_SUBSCRIPTION(MrmState);
  CONCEALER_DEFINE_SUBSCRIPTION(PathWithLaneId);
  CONCEALER_DEFINE_SUBSCRIPTION(Trajectory);
  CONCEALER_DEFINE_SUBSCRIPTION(TurnIndicatorsCommand, override);

  using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
  using Engage = tier4_external_api_msgs::srv::Engage;
  // TODO using InitializePose = tier4_external_api_msgs::srv::InitializePose;
  using SetVelocityLimit = tier4_external_api_msgs::srv::SetVelocityLimit;

  ServiceWithValidation<CooperateCommands> requestCooperateCommands;
  ServiceWithValidation<Engage> requestEngage;
  // TODO ServiceWithValidation<InitializePose> requestInitializePose;
  ServiceWithValidation<SetVelocityLimit> requestSetVelocityLimit;

private:
  Cooperator current_cooperator = Cooperator::simulator;

  TaskQueue cooperation_queue;

  auto approve(const CooperateStatusArray &) -> void;

  auto cooperate(const CooperateStatusArray &) -> void;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  auto receiveMrmState(const MrmState & msg) -> void;

  auto receiveEmergencyState(const EmergencyState & msg) -> void;

public:
#define DEFINE_STATE_PREDICATE(NAME, VALUE)                  \
  auto is##NAME() const noexcept                             \
  {                                                          \
    using autoware_auto_system_msgs::msg::AutowareState;     \
    return getAutowareState().state == AutowareState::VALUE; \
  }                                                          \
  static_assert(true, "")

  DEFINE_STATE_PREDICATE(Initializing, INITIALIZING);            // 1
  DEFINE_STATE_PREDICATE(WaitingForRoute, WAITING_FOR_ROUTE);    // 2
  DEFINE_STATE_PREDICATE(Planning, PLANNING);                    // 3
  DEFINE_STATE_PREDICATE(WaitingForEngage, WAITING_FOR_ENGAGE);  // 4
  DEFINE_STATE_PREDICATE(Driving, DRIVING);                      // 5
  DEFINE_STATE_PREDICATE(ArrivedGoal, ARRIVED_GOAL);             // 6
  DEFINE_STATE_PREDICATE(Finalizing, FINALIZING);                // 7

#undef DEFINE_STATE_PREDICATE

  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverse(Ts &&... xs)
  : Autoware(std::forward<decltype(xs)>(xs)...),
    // clang-format off
    CONCEALER_INIT_PUBLISHER(Acceleration, "/localization/acceleration"),
    CONCEALER_INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    CONCEALER_INIT_PUBLISHER(ControlModeReport, "/vehicle/status/control_mode"),
    CONCEALER_INIT_PUBLISHER(GearReport, "/vehicle/status/gear_status"),
    CONCEALER_INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    CONCEALER_INIT_PUBLISHER(InitialPose, "/initialpose"),
    CONCEALER_INIT_PUBLISHER(Odometry, "/localization/kinematic_state"),
    CONCEALER_INIT_PUBLISHER(SteeringReport, "/vehicle/status/steering_status"),
    CONCEALER_INIT_PUBLISHER(TurnIndicatorsReport, "/vehicle/status/turn_indicators_status"),
    CONCEALER_INIT_PUBLISHER(VelocityReport, "/vehicle/status/velocity_status"),
    CONCEALER_INIT_SUBSCRIPTION(AckermannControlCommand, "/control/command/control_cmd"),
    CONCEALER_INIT_SUBSCRIPTION(AutowareState, "/autoware/state"),
    CONCEALER_INIT_SUBSCRIPTION_WITH_CALLBACK(CooperateStatusArray, "/api/external/get/rtc_status", cooperate),
    CONCEALER_INIT_SUBSCRIPTION_WITH_CALLBACK(EmergencyState, "/system/emergency/emergency_state", receiveEmergencyState),
    CONCEALER_INIT_SUBSCRIPTION(GearCommand, "/control/command/gear_cmd"),
    CONCEALER_INIT_SUBSCRIPTION_WITH_CALLBACK(MrmState, "/api/fail_safe/mrm_state", receiveMrmState),
    CONCEALER_INIT_SUBSCRIPTION(PathWithLaneId, "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id"),
    CONCEALER_INIT_SUBSCRIPTION(Trajectory, "/planning/scenario_planning/trajectory"),
    CONCEALER_INIT_SUBSCRIPTION(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd"),
    requestCooperateCommands("/api/external/set/rtc_commands", *this),
    requestEngage("/api/external/set/engage", *this),
    // TODO requestInitializePose("/api/autoware/set/initialize_pose", *this),
    requestSetVelocityLimit("/api/autoware/set/velocity_limit", *this)
  // clang-format on
  {
    waitpid_options = 0;

    resetTimerCallback();
  }

  ~AutowareUniverse() override;

  auto engage() -> void override;

  auto engageable() const -> bool override;

  auto engaged() const -> bool override;

  auto getAcceleration() const -> double override;

  auto getAutowareStateName() const -> std::string override;

  auto getEmergencyStateName() const -> std::string override;

  auto getGearSign() const -> double override;

  auto getMinimumRiskManeuverBehaviorName() const -> std::string override;

  auto getMinimumRiskManeuverStateName() const -> std::string override;

  auto getSteeringAngle() const -> double override;

  auto getVehicleCommand() const -> std::tuple<
    autoware_auto_control_msgs::msg::AckermannControlCommand,
    autoware_auto_vehicle_msgs::msg::GearCommand> override;

  auto getVelocity() const -> double override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto sendSIGINT() -> void override;

  auto setCooperator(const std::string & cooperator) -> void override
  {
    current_cooperator = boost::lexical_cast<Cooperator>(cooperator);
  }

  auto setVelocityLimit(double) -> void override;

  auto update() -> void override;
};
}  // namespace concealer

// for boost::lexical_cast
namespace autoware_auto_vehicle_msgs::msg
{
auto operator<<(std::ostream &, const TurnIndicatorsCommand &) -> std::ostream &;

auto operator>>(std::istream &, TurnIndicatorsCommand &) -> std::istream &;
}  // namespace autoware_auto_vehicle_msgs::msg

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
