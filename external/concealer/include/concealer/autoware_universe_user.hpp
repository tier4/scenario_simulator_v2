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

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <concealer/autoware_user.hpp>
#include <concealer/cooperator.hpp>
#include <concealer/task_queue.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
// TODO #include <tier4_external_api_msgs/srv/initialize_pose.hpp>
#include <concealer/utility/publisher_wrapper.hpp>
#include <concealer/utility/service_with_validation.hpp>
#include <concealer/utility/subscriber_wrapper.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

namespace concealer
{
class AutowareUniverseUser : public AutowareUser, public TransitionAssertion<AutowareUniverseUser>
{
  friend class TransitionAssertion<AutowareUniverseUser>;

  using Checkpoint = geometry_msgs::msg::PoseStamped;
  using GoalPose = geometry_msgs::msg::PoseStamped;
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;

  PublisherWrapper<Checkpoint> setCheckpoint;
  PublisherWrapper<GoalPose> setGoalPose;
  PublisherWrapper<InitialPose> setInitialPose;

  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;
  using EmergencyState = autoware_auto_system_msgs::msg::EmergencyState;
  using MrmState = autoware_adapi_v1_msgs::msg::MrmState;
  using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  SubscriberWrapper<AutowareState, ThreadSafe> getAutowareState;
  SubscriberWrapper<AckermannControlCommand> getAckermannControlCommand;
  SubscriberWrapper<CooperateStatusArray> getCooperateStatusArray;
  SubscriberWrapper<EmergencyState> getEmergencyState;
  SubscriberWrapper<MrmState> getMrmState;
  SubscriberWrapper<Trajectory> getTrajectory;
  SubscriberWrapper<TurnIndicatorsCommand> getTurnIndicatorsCommandImpl;

  using Engage = tier4_external_api_msgs::srv::Engage;
  // TODO using InitializePose = tier4_external_api_msgs::srv::InitializePose;
  using SetVelocityLimit = tier4_external_api_msgs::srv::SetVelocityLimit;

  ServiceWithValidation<Engage> requestEngage;
  // TODO ServiceWithValidation<InitializePose> requestInitializePose;
  ServiceWithValidation<SetVelocityLimit> requestSetVelocityLimit;

  using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
  ServiceWithValidation<CooperateCommands> requestCooperateCommands;
  Cooperator current_cooperator = Cooperator::simulator;

  auto approve(const CooperateStatusArray &) -> void;
  auto cooperate(const CooperateStatusArray &) -> void;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  auto receiveMrmState(const MrmState & msg) -> void;

  auto receiveEmergencyState(const EmergencyState & msg) -> void;

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

protected:
  auto sendSIGINT() -> void override;

public:
  SubscriberWrapper<PathWithLaneId> getPathWithLaneId;

public:
  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverseUser(Ts &&... xs)
  : AutowareUser(std::forward<decltype(xs)>(xs)...),
    // clang-format off
    setCheckpoint("/planning/mission_planning/checkpoint", *this),
    setGoalPose("/planning/mission_planning/goal", *this),
    setInitialPose("/initialpose", *this),
    getAutowareState("/autoware/state", *this),
    getAckermannControlCommand("/control/command/control_cmd", *this),
    getCooperateStatusArray("/api/external/get/rtc_status", *this, [this](const CooperateStatusArray& v) {cooperate(v);}),
    getEmergencyState("/system/emergency/emergency_state", *this, [this](const EmergencyState& v) {receiveEmergencyState(v);}),
    getMrmState("/api/fail_safe/mrm_state", *this, [this](const MrmState& v){receiveMrmState(v);}),
    getTrajectory("/planning/scenario_planning/trajectory", *this),
    getTurnIndicatorsCommandImpl("/control/command/turn_indicators_cmd", *this),
    requestEngage("/api/external/set/engage", *this),
    // TODO requestInitializePose("/api/autoware/set/initialize_pose", *this),
    requestSetVelocityLimit("/api/autoware/set/velocity_limit", *this),
    requestCooperateCommands("/api/external/set/rtc_commands", *this),
    getPathWithLaneId("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", *this)
  // clang-format on
  {
  }

  ~AutowareUniverseUser() override;

  auto engage() -> void override;

  auto engageable() const -> bool override;

  auto engaged() const -> bool override;

  auto getAutowareStateName() const -> std::string override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto getTurnIndicatorsCommand() const
    -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand override;

  auto getEmergencyStateName() const -> std::string override;

  auto getMinimumRiskManeuverBehaviorName() const -> std::string override;

  auto getMinimumRiskManeuverStateName() const -> std::string override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto setCooperator(const std::string & cooperator) -> void override;

  auto setVelocityLimit(double) -> void override;
};
}  // namespace concealer

namespace autoware_auto_vehicle_msgs::msg
{
auto operator<<(std::ostream &, const TurnIndicatorsCommand &) -> std::ostream &;

auto operator>>(std::istream &, TurnIndicatorsCommand &) -> std::istream &;
}  // namespace autoware_auto_vehicle_msgs::msg

#endif  // CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_
