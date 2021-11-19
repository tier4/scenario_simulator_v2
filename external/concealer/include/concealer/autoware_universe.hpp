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

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_api_msgs/msg/velocity_limit.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/lane_change_command.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
// #include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
// #include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
// #include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
// #include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
// #include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <boost/range/adaptor/sliced.hpp>
#include <concealer/autoware.hpp>
#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace concealer
{
class AutowareUniverse : public Autoware, public TransitionAssertion<AutowareUniverse>
{
  friend class TransitionAssertion<AutowareUniverse>;

  void sendSIGINT() override;

  bool isReady() noexcept;

  bool isNotReady() noexcept;

  void checkAutowareState();

  bool is_ready = false;

  /// FROM MiscellaneousAPI ///
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* ---- Checkpoint -----------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/checkpoint
   *
   * ------------------------------------------------------------------------ */
  using Checkpoint = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(Checkpoint);

  /* ---- CurrentControlMode ---------------------------------------------------
   *
   *  Topic: /vehicle/status/control_mode
   *
   * ------------------------------------------------------------------------ */
  using CurrentControlMode = autoware_auto_vehicle_msgs::msg::ControlModeReport;

  DEFINE_PUBLISHER(CurrentControlMode);

  decltype(auto) setCurrentControlMode(const std::uint8_t mode = CurrentControlMode::AUTONOMOUS)
  {
    CurrentControlMode current_control_mode{};
    {
      current_control_mode.mode = mode;
    }

    return setCurrentControlMode(current_control_mode);
  }

  /* ---- CurrentShift ---------------------------------------------------------
   *
   *  Topic: /vehicle/status/gear_status
   *
   *  Overloads:
   *    setCurrentShift(const autoware_auto_vehicle_msgs::msg::GearReport &);
   *    setCurrentShift(const double);
   *
   * ------------------------------------------------------------------------ */
  using CurrentShift = autoware_auto_vehicle_msgs::msg::GearReport;

  DEFINE_PUBLISHER(CurrentShift);

  template <typename T, REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentShift(const T twist_linear_x)
  {
    CurrentShift current_shift;
    {
      using autoware_auto_vehicle_msgs::msg::GearReport;

      current_shift.stamp = get_clock()->now();
      current_shift.report = twist_linear_x >= 0 ? GearReport::DRIVE : GearReport::REVERSE;
    }

    return setCurrentShift(current_shift);
  }

  decltype(auto) setCurrentShift(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentShift(twist.linear.x);
  }

  /* ---- CurrentSteering ------------------------------------------------------
   *
   *  Topic: /vehicle/status/steering_status
   *
   * ------------------------------------------------------------------------ */
  // TODO(murooka) steering is tire angle, not tire angular velocity
  /*
  using CurrentSteering = autoware_auto_vehicle_msgs::msg::SteeringReport;

  DEFINE_PUBLISHER(CurrentSteering);

  template <typename T, REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentSteering(const T value)
  {
    CurrentSteering current_steering{};
    {
      current_steering.stamp = get_clock()->now();
      current_steering.report = value;
    }

    return setCurrentSteering(current_steering);
  }

  decltype(auto) setCurrentSteering(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentSteering(twist.angular.z);
  }
  */

  /* ---- CurrentTurnSignal ----------------------------------------------------
   *
   *  Topic: /vehicle/status/turn_signal
   *
   * ------------------------------------------------------------------------ */
  using CurrentTurnIndicators = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;

  DEFINE_PUBLISHER(CurrentTurnIndicators);

  decltype(auto) setCurrentTurnIndicators()
  {
    CurrentTurnIndicators current_turn_signal{};
    {
      current_turn_signal.stamp = get_clock()->now();
      // TODO(murooka) there is no NONE in turn indicators report msg
      // current_turn_signal.report = autoware_vehicle_msgs::msg::TurnIndicators::NONE;
    }

    return setCurrentTurnIndicators(current_turn_signal);

    // return setCurrentTurnIndicators(getTurnIndicatorsCommand());
  }

  /* ---- CurrentHazardLights ----------------------------------------------------
   *
   *  Topic: /vehicle/status/hazard_lights_status
   *
   * ------------------------------------------------------------------------ */
  using CurrentHazardLights = autoware_auto_vehicle_msgs::msg::HazardLightsReport;

  DEFINE_PUBLISHER(CurrentHazardLights);

  decltype(auto) setCurrentHazardLights()
  {
    CurrentHazardLights current_turn_signal{};
    {
      current_turn_signal.stamp = get_clock()->now();
      // TODO(murooka) there is no NONE in hazard lights report msg
      // current_turn_signal.report = autoware_vehicle_msgs::msg::HazardLights::NONE;
    }

    return setCurrentHazardLights(current_turn_signal);

    // return setCurrentHazardLights(getHazardLightsCommand());
  }

  /* ---- CurrentTwist ---------------------------------------------------------
   *
   *  Topic: /vehicle/status/twist
   *
   * ------------------------------------------------------------------------ */
  using CurrentTwist = geometry_msgs::msg::TwistStamped;

  DEFINE_PUBLISHER(CurrentTwist);

  decltype(auto) setCurrentTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::TwistStamped current_twist{};
    {
      current_twist.header.stamp = get_clock()->now();
      current_twist.header.frame_id = "map";
      current_twist.twist = twist;
    }

    return setCurrentTwist(current_twist);
  }

  /* ---- CurrentVelocity ------------------------------------------------------
   *
   *  Topic: /vehicle/status/velocity_status
   *
   * ------------------------------------------------------------------------ */
  using CurrentVelocity = autoware_auto_vehicle_msgs::msg::VelocityReport;

  DEFINE_PUBLISHER(CurrentVelocity);

  /*
  template <typename T, REQUIRES(std::is_convertible<T, decltype(CurrentVelocity::data)>)>
  decltype(auto) setCurrentVelocity(const T velocity)
  {
    CurrentVelocity message;
    {
      message.header.stamp = get_clock()->now();
      message.header.frame_id = velocity.header.frame_id;
      message.longitudinal_velocity = twist.linear.x;
    }

    return setCurrentVelocity(message);
  }
  */

  decltype(auto) setCurrentVelocity(const geometry_msgs::msg::Twist & twist)
  {
    CurrentVelocity message{};
    {
      message.header.stamp = get_clock()->now();
      message.header.frame_id = "base_link";  // TODO(murooka) check if base_link is correct
      message.longitudinal_velocity = twist.linear.x;
      message.lateral_velocity = twist.linear.y;
      message.heading_rate = twist.angular.z;
    }

    return setCurrentVelocity(message);
  }

  /* ---- GoalPose -------------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/goal
   *
   * ------------------------------------------------------------------------ */
  using GoalPose = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(GoalPose);

  /* ---- InitialPose ----------------------------------------------------------
   *
   *  Set initial pose of Autoware.
   *
   *  Topic: /initialpose
   *
   * ------------------------------------------------------------------------ */
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;

  DEFINE_PUBLISHER(InitialPose);

  decltype(auto) setInitialPose(const geometry_msgs::msg::Pose & pose)
  {
    InitialPose initial_pose;
    {
      initial_pose.header.stamp = get_clock()->now();
      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose = pose;
    }

    return setInitialPose(initial_pose);
  }

  /* ---- LocalizationPose -----------------------------------------------------
    *
    *  Topic: /localization/pose_with_covariance
    *
    * ------------------------------------------------------------------------ */
  /*
  using LocalizationPose = geometry_msgs::msg::PoseWithCovarianceStamped;

  DEFINE_PUBLISHER(LocalizationPose);

  auto setLocalizationPose(
    const geometry_msgs::msg::Pose & pose = geometry_msgs::msg::Pose(),
    const std::array<double, 36> & covariance = {}) -> decltype(auto)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped;
    {
      pose_with_covariance_stamped.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      pose_with_covariance_stamped.header.frame_id = "map";
      pose_with_covariance_stamped.pose.pose = pose;
      pose_with_covariance_stamped.pose.covariance = covariance;
    }

    return setLocalizationPose(pose_with_covariance_stamped);
  }
  */

  /* ---- LocalizationTwist ----------------------------------------------------
   *
   *  Topic: /localization/twist
   *
   * ------------------------------------------------------------------------ */
  /*
  using LocalizationTwist = CurrentTwist;

  DEFINE_PUBLISHER(LocalizationTwist);

  decltype(auto) setLocalizationTwist(
    const geometry_msgs::msg::Twist & twist = geometry_msgs::msg::Twist())
  {
    LocalizationTwist localization_twist;
    {
      localization_twist.header.stamp = get_clock()->now();
      localization_twist.header.frame_id = "map";
      localization_twist.twist = twist;
    }

    return setLocalizationTwist(localization_twist);
  }
  */

  /* ---- LocalizationOdometry ----------------------------------------------------
   *
   *  Topic: /localization/kinematic_state
   *
   * ------------------------------------------------------------------------ */
  using LocalizationOdometry = nav_msgs::msg::Odometry;

  DEFINE_PUBLISHER(LocalizationOdometry);

  decltype(auto) setLocalizationOdometry(
    const geometry_msgs::msg::Pose & pose = geometry_msgs::msg::Pose(),
    const geometry_msgs::msg::Twist & twist = geometry_msgs::msg::Twist())
  {
    LocalizationOdometry localization_odometry;
    {
      localization_odometry.header.stamp = get_clock()->now();
      localization_odometry.header.frame_id = "map";
      localization_odometry.pose.pose = pose;
      localization_odometry.twist.twist = twist;
    }

    return setLocalizationOdometry(localization_odometry);
  }

  /* ---- Trajectory -----------------------------------------------------------
   *
   *  Topic: /planning/scenario_planning/trajectory
   *
   * ------------------------------------------------------------------------ */
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  DEFINE_SUBSCRIPTION(Trajectory);

  /* ---- Gear Command --------------------------------------------------
   *
   *  Topic: /control/command/gear_cmd
   *
   * ------------------------------------------------------------------------ */
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;

  DEFINE_SUBSCRIPTION(GearCommand);

  /* ---- Turn Signal Command --------------------------------------------------
   *
   *  Topic: /control/turn_signal_cmd
   *
   * ------------------------------------------------------------------------ */
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  DEFINE_SUBSCRIPTION(TurnIndicatorsCommand);

  /* ---- Hazard Lights Command --------------------------------------------------
   *
   *  Topic: /control/turn_signal_cmd
   *
   * ------------------------------------------------------------------------ */
  using HazardLightsCommand = autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

  DEFINE_SUBSCRIPTION(HazardLightsCommand);

  /* ---- AckermannControlCommand -------------------------------------------------------
   *
   *  Topic: /control/command/control_cmd
   *
   * ------------------------------------------------------------------------ */
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;

  // this macro is used only once
  // however is is created to keep consistency with the previous DEFINE_SUBSCRIPTION macro usage
  // DEFINE_SUBSCRIPTION_WITH_OVERRIDE(AckermannControlCommand);
  DEFINE_SUBSCRIPTION(AckermannControlCommand);  // TODO(murooka) is this implementation correct?
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  autoware_vehicle_msgs::msg::VehicleCommand getVehicleCommand() const override;

public:
  /// FROM FundamentalAPI ///
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* ---- AutowareEngage -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/engage
   *
   *  Overloads:
   *    setAutowareEngage(const AutowareEngage &) const
   *    setAutowareEngage(const bool) const
   *
   * ------------------------------------------------------------------------ */
  using AutowareEngage = autoware_auto_vehicle_msgs::msg::Engage;

  DEFINE_PUBLISHER(AutowareEngage);

  decltype(auto) setAutowareEngage(const bool value = true)
  {
    // return setAutowareEngage(convertTo<AutowareEngage>(value));
    AutowareEngage message{};
    {
      message.stamp = get_clock()->now();
      message.engage = value;
    }

    return setAutowareEngage(message);
  }

  /* ---- AutowareRoute --------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/route
   *
   * ------------------------------------------------------------------------ */
  // using AutowareRoute = autoware_planning_msgs::msg::Route;
  //
  // DEFINE_PUBLISHER(AutowareRoute);

  /* ---- LaneChangeApproval ---------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/approval
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeApproval = autoware_planning_msgs::msg::LaneChangeCommand;

  DEFINE_PUBLISHER(LaneChangeApproval);

  decltype(auto) setLaneChangeApproval(const bool approve = true)
  {
    LaneChangeForce message;
    {
      message.stamp = get_clock()->now();
      message.command = approve;
    }

    return setLaneChangeApproval(message);
  }

  /* ---- LaneChangeForce ------------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/force
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeForce = autoware_planning_msgs::msg::LaneChangeCommand;

  DEFINE_PUBLISHER(LaneChangeForce);

  decltype(auto) setLaneChangeForce(const bool force = true)
  {
    LaneChangeForce message;
    {
      message.stamp = get_clock()->now();
      message.command = force;
    }

    return setLaneChangeForce(message);
  }

  /* ---- TrafficLightStateArray -----------------------------------------------
   *
   *  Overwrite the recognition result of traffic light.
   *
   *  Topic: /awapi/traffic_light/put/traffic_light
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStateArray = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_PUBLISHER(TrafficLightStateArray);

  /* ---- VehicleVelocity ------------------------------------------------------
   *
   *  Set upper bound of velocity.
   *
   *  Topic: /awapi/vehicle/put/velocity
   *
   * ------------------------------------------------------------------------ */
  using VehicleVelocity = autoware_api_msgs::msg::VelocityLimit;

  DEFINE_PUBLISHER(VehicleVelocity);

  template <typename T, REQUIRES(std::is_convertible<T, decltype(VehicleVelocity::max_velocity)>)>
  auto setVehicleVelocity(const T & value) -> decltype(auto)
  {
    VehicleVelocity vehicle_velocity;
    {
      vehicle_velocity.stamp = get_clock()->now();
      vehicle_velocity.max_velocity = value;
    }

    return setVehicleVelocity(vehicle_velocity);
  }

  /* ---- AutowareStatus -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/get/status
   *
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;

  DEFINE_SUBSCRIPTION(AutowareStatus);

  /* ---- TrafficLightStatus ---------------------------------------------------
   *
   *  Topic: /awapi/traffic_light/get/status
   *
   * ------------------------------------------------------------------------ */
  // using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;
  //
  // DEFINE_SUBSCRIPTION(TrafficLightStatus);

  /* ---- VehicleStatus --------------------------------------------------------
   *
   *  Topic: /awapi/vehicle/get/status
   *
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

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
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverse(Ts &&... xs)
  : Autoware(std::forward<decltype(xs)>(xs)...),
    /// MiscellaneousAPI
    INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(CurrentControlMode, "/vehicle/status/control_mode"),
    INIT_PUBLISHER(CurrentShift, "/vehicle/status/gear_status"),
    // INIT_PUBLISHER(CurrentSteering, "/vehicle/status/steering_status"),
    INIT_PUBLISHER(CurrentTurnIndicators, "/vehicle/status/turn_indicators_status"),
    INIT_PUBLISHER(CurrentHazardLights, "/vehicle/status/hazard_lights_status"),
    // INIT_PUBLISHER(CurrentTwist, "/vehicle/status/twist"),
    INIT_PUBLISHER(CurrentVelocity, "/vehicle/status/velocity_status"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    // INIT_PUBLISHER(LocalizationPose, "/localization/pose_with_covariance"),
    // INIT_PUBLISHER(LocalizationTwist, "/localization/twist"),
    INIT_PUBLISHER(LocalizationOdometry, "/localization/kinematic_state"),
    INIT_SUBSCRIPTION(Trajectory, "/planning/scenario_planning/trajectory", []() {}),
    INIT_SUBSCRIPTION(GearCommand, "/control/command/gear_cmd", []() {}),
    INIT_SUBSCRIPTION(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", []() {}),
    INIT_SUBSCRIPTION(HazardLightsCommand, "/control/command/hazard_lights_cmd", []() {}),
    INIT_SUBSCRIPTION(AckermannControlCommand, "/control/command/control_cmd", []() {}),
    /// FundamentalAPI
    INIT_PUBLISHER(AutowareEngage, "/awapi/autoware/put/engage"),
    // INIT_PUBLISHER(AutowareRoute, "/awapi/autoware/put/route"),
    INIT_PUBLISHER(LaneChangeApproval, "/awapi/lane_change/put/approval"),
    INIT_PUBLISHER(LaneChangeForce, "/awapi/lane_change/put/force"),
    INIT_PUBLISHER(TrafficLightStateArray, "/awapi/traffic_light/put/traffic_light_status"),
    INIT_PUBLISHER(VehicleVelocity, "/awapi/vehicle/put/velocity"),
    INIT_SUBSCRIPTION(AutowareStatus, "/awapi/autoware/get/status", checkAutowareState),
    // INIT_SUBSCRIPTION(TrafficLightStatus, "/awapi/traffic_light/get/status", []() {}),
    INIT_SUBSCRIPTION(VehicleStatus, "/awapi/vehicle/get/status", []() {})
  {
    waitpid_options = 0;

    resetTimerCallback();
    setLaneChangeApproval();
  }

  virtual ~AutowareUniverse();

  auto engage() -> void override;

  auto getAcceleration() const -> double override;

  auto getAutowareStateMessage() const -> std::string override;

  auto getGearSign() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVelocity() const -> double override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto update() -> void override;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
