// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_api_msgs/msg/velocity_limit.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_planning_msgs/msg/lane_change_command.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <concealer/autoware.hpp>
#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace concealer
{
class AutowareUniverse : public Autoware, public TransitionAssertion<AutowareUniverse>
{
  friend class TransitionAssertion<AutowareUniverse>;

  bool is_ready = false;

  using AutowareEngage = autoware_auto_vehicle_msgs::msg::Engage;
  using Checkpoint = geometry_msgs::msg::PoseStamped;
  using CurrentControlMode = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using CurrentHazardLights = autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using CurrentShift = autoware_auto_vehicle_msgs::msg::GearReport;
  using CurrentSteering = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using CurrentTurnIndicators = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using CurrentTwist = geometry_msgs::msg::TwistStamped;
  using CurrentVelocity = autoware_auto_vehicle_msgs::msg::VelocityReport;
  using GoalPose = geometry_msgs::msg::PoseStamped;
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using LaneChangeApproval = autoware_planning_msgs::msg::LaneChangeCommand;
  using LocalizationOdometry = nav_msgs::msg::Odometry;
  using VehicleVelocity = autoware_api_msgs::msg::VelocityLimit;

  DEFINE_PUBLISHER(AutowareEngage);
  DEFINE_PUBLISHER(Checkpoint);
  DEFINE_PUBLISHER(CurrentControlMode);
  DEFINE_PUBLISHER(CurrentHazardLights);
  DEFINE_PUBLISHER(CurrentShift);
  DEFINE_PUBLISHER(CurrentSteering);
  DEFINE_PUBLISHER(CurrentTurnIndicators);
  DEFINE_PUBLISHER(CurrentTwist);
  DEFINE_PUBLISHER(CurrentVelocity);
  DEFINE_PUBLISHER(GoalPose);
  DEFINE_PUBLISHER(InitialPose);
  DEFINE_PUBLISHER(LaneChangeApproval);
  DEFINE_PUBLISHER(LocalizationOdometry);
  DEFINE_PUBLISHER(VehicleVelocity);

  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
  using HazardLightsCommand = autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

  DEFINE_SUBSCRIPTION(AckermannControlCommand);
  DEFINE_SUBSCRIPTION(AutowareStatus);
  DEFINE_SUBSCRIPTION(GearCommand);
  DEFINE_SUBSCRIPTION(HazardLightsCommand);
  DEFINE_SUBSCRIPTION(Trajectory);
  DEFINE_SUBSCRIPTION(TurnIndicatorsCommand);
  DEFINE_SUBSCRIPTION(VehicleStatus);

public:
#define DEFINE_STATE_PREDICATE(NAME, VALUE)                                                   \
  auto is##NAME() const noexcept                                                              \
  {                                                                                           \
    using autoware_system_msgs::msg::AutowareState;                                           \
    assert(AutowareState::VALUE == #NAME);                                                    \
    return CONCEALER_CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::VALUE; \
  }                                                                                           \
  static_assert(true, "")

  DEFINE_STATE_PREDICATE(InitializingVehicle, INITIALIZING_VEHICLE);
  DEFINE_STATE_PREDICATE(WaitingForRoute, WAITING_FOR_ROUTE);
  DEFINE_STATE_PREDICATE(Planning, PLANNING);
  DEFINE_STATE_PREDICATE(WaitingForEngage, WAITING_FOR_ENGAGE);
  DEFINE_STATE_PREDICATE(Driving, DRIVING);
  DEFINE_STATE_PREDICATE(ArrivedGoal, ARRIVAL_GOAL);
  DEFINE_STATE_PREDICATE(Emergency, EMERGENCY);
  DEFINE_STATE_PREDICATE(Finalizing, FINALIZING);

#undef DEFINE_STATE_PREDICATE

  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverse(Ts &&... xs)
  : Autoware(std::forward<decltype(xs)>(xs)...),
    INIT_PUBLISHER(AutowareEngage, "/awapi/autoware/put/engage"),
    INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(CurrentControlMode, "/vehicle/status/control_mode"),
    INIT_PUBLISHER(CurrentHazardLights, "/vehicle/status/hazard_lights_status"),
    INIT_PUBLISHER(CurrentShift, "/vehicle/status/gear_status"),
    INIT_PUBLISHER(CurrentSteering, "/vehicle/status/steering_status"),
    INIT_PUBLISHER(CurrentTurnIndicators, "/vehicle/status/turn_indicators_status"),
    INIT_PUBLISHER(CurrentVelocity, "/vehicle/status/velocity_status"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    INIT_PUBLISHER(LaneChangeApproval, "/awapi/lane_change/put/approval"),
    INIT_PUBLISHER(LocalizationOdometry, "/localization/kinematic_state"),
    INIT_PUBLISHER(VehicleVelocity, "/awapi/vehicle/put/velocity"),
    INIT_SUBSCRIPTION(AckermannControlCommand, "/control/command/control_cmd", []() {}),
    INIT_SUBSCRIPTION(AutowareStatus, "/awapi/autoware/get/status", checkAutowareState),
    INIT_SUBSCRIPTION(GearCommand, "/control/command/gear_cmd", []() {}),
    INIT_SUBSCRIPTION(HazardLightsCommand, "/control/command/hazard_lights_cmd", []() {}),
    INIT_SUBSCRIPTION(Trajectory, "/planning/scenario_planning/trajectory", []() {}),
    INIT_SUBSCRIPTION(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", []() {}),
    INIT_SUBSCRIPTION(VehicleStatus, "/awapi/vehicle/get/status", []() {})
  {
    waitpid_options = 0;

    resetTimerCallback();

    LaneChangeApproval lane_change_approval;
    {
      lane_change_approval.stamp = get_clock()->now();
      lane_change_approval.command = true;
    }
    setLaneChangeApproval(lane_change_approval);
  }

  virtual ~AutowareUniverse();

  auto checkAutowareState() -> void;

  auto engage() -> void override;

  auto getAcceleration() const -> double override;

  auto getAutowareStateMessage() const -> std::string override;

  auto getGearSign() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVehicleCommand() const -> autoware_vehicle_msgs::msg::VehicleCommand override;

  auto getVelocity() const -> double override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto isReady() noexcept -> bool;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto sendSIGINT() -> void override;

  auto update() -> void override;
};
}  // namespace concealer

#else  // ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO

#include <concealer/autoware.hpp>

namespace concealer
{
struct AutowareUniverse : public Autoware
{
  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverse(Ts &&... xs)
  : Autoware(std::forward<decltype(xs)>(xs)...)
  {
  }

  virtual ~AutowareUniverse();

  auto engage() -> void override;

  auto getAcceleration() const -> double override;

  auto getAutowareStateMessage() const -> std::string override;

  auto getGearSign() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVehicleCommand() const -> autoware_vehicle_msgs::msg::VehicleCommand override;

  auto getVelocity() const -> double override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto sendSIGINT() -> void override;

  auto update() -> void override;
};
}  // namespace concealer

#endif  // SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
