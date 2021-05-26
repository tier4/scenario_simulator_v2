// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__MISCELLANEOUS_API_HPP_
#define CONCEALER__MISCELLANEOUS_API_HPP_

#include <concealer/autoware_def.hpp>

#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#endif

#ifdef AUTOWARE_AUTO
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#endif

namespace concealer
{
template <typename Node>
class MiscellaneousAPI
{
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
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
  using CurrentControlMode = autoware_vehicle_msgs::msg::ControlMode;

  DEFINE_PUBLISHER(CurrentControlMode);

  decltype(auto) setCurrentControlMode(const std::uint8_t mode = CurrentControlMode::AUTO)
  {
    CurrentControlMode current_control_mode{};
    {
      current_control_mode.data = mode;
    }

    return setCurrentControlMode(current_control_mode);
  }

  /* ---- CurrentShift ---------------------------------------------------------
   *
   *  Topic: /vehicle/status/shift
   *
   *  Overloads:
   *    setCurrentShift(const autoware_vehicle_msgs::msg::ShiftStamped &);
   *    setCurrentShift(const double);
   *
   * ------------------------------------------------------------------------ */
  using CurrentShift = autoware_vehicle_msgs::msg::ShiftStamped;

  DEFINE_PUBLISHER(CurrentShift);

  template <typename T, REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentShift(const T twist_linear_x)
  {
    CurrentShift current_shift;
    {
      using autoware_vehicle_msgs::msg::Shift;

      current_shift.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      current_shift.header.frame_id = "map";
      current_shift.shift.data = twist_linear_x >= 0 ? Shift::DRIVE : Shift::REVERSE;
    }

    return setCurrentShift(current_shift);
  }

  decltype(auto) setCurrentShift(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentShift(twist.linear.x);
  }

  /* ---- CurrentSteering ------------------------------------------------------
   *
   *  Topic: /vehicle/status/steering
   *
   * ------------------------------------------------------------------------ */
  using CurrentSteering = autoware_vehicle_msgs::msg::Steering;

  DEFINE_PUBLISHER(CurrentSteering);

  template <typename T, REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentSteering(const T value)
  {
    CurrentSteering current_steering{};
    {
      current_steering.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      current_steering.header.frame_id = "base_link";
      current_steering.data = value;
    }

    return setCurrentSteering(current_steering);
  }

  decltype(auto) setCurrentSteering(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentSteering(twist.angular.z);
  }

  /* ---- CurrentTurnSignal ----------------------------------------------------
   *
   *  Topic: /vehicle/status/turn_signal
   *
   * ------------------------------------------------------------------------ */
  using CurrentTurnSignal = autoware_vehicle_msgs::msg::TurnSignal;

  DEFINE_PUBLISHER(CurrentTurnSignal);

  decltype(auto) setCurrentTurnSignal()
  {
    CurrentTurnSignal current_turn_signal{};
    {
      current_turn_signal.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      current_turn_signal.header.frame_id = "map";
      current_turn_signal.data = autoware_vehicle_msgs::msg::TurnSignal::NONE;
    }

    return setCurrentTurnSignal(current_turn_signal);

    // return setCurrentTurnSignal(getTurnSignalCommand());
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
      current_twist.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      current_twist.header.frame_id = "map";
      current_twist.twist = twist;
    }

    return setCurrentTwist(current_twist);
  }

  /* ---- CurrentVelocity ------------------------------------------------------
   *
   *  Topic: /vehicle/status/velocity
   *
   * ------------------------------------------------------------------------ */
  using CurrentVelocity = autoware_debug_msgs::msg::Float32Stamped;

  DEFINE_PUBLISHER(CurrentVelocity);

  template <typename T, REQUIRES(std::is_convertible<T, decltype(CurrentVelocity::data)>)>
  decltype(auto) setCurrentVelocity(const T twist_linear_x)
  {
    CurrentVelocity message;
    {
      message.stamp = static_cast<Node &>(*this).get_clock()->now();
      message.data = twist_linear_x;
    }

    return setCurrentVelocity(message);
  }

  decltype(auto) setCurrentVelocity(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentVelocity(twist.linear.x);
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
      initial_pose.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose = pose;
    }

    return setInitialPose(initial_pose);
  }

  /* ---- LocalizationTwist ----------------------------------------------------
   *
   *  Topic: /localization/twist
   *
   * ------------------------------------------------------------------------ */
  using LocalizationTwist = CurrentTwist;

  DEFINE_PUBLISHER(LocalizationTwist);

  decltype(auto) setLocalizationTwist(
    const geometry_msgs::msg::Twist & twist = geometry_msgs::msg::Twist())
  {
    LocalizationTwist localization_twist;
    {
      localization_twist.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      localization_twist.header.frame_id = "map";
      localization_twist.twist = twist;
    }

    return setLocalizationTwist(localization_twist);
  }

  /* ---- Trajectory -----------------------------------------------------------
   *
   *  Topic: /planning/scenario_planning/trajectory
   *
   * ------------------------------------------------------------------------ */
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  DEFINE_SUBSCRIPTION(Trajectory);

  /* ---- Turn Signal Command --------------------------------------------------
   *
   *  Topic: /control/turn_signal_cmd
   *
   * ------------------------------------------------------------------------ */
  using TurnSignalCommand = autoware_vehicle_msgs::msg::TurnSignal;

  DEFINE_SUBSCRIPTION(TurnSignalCommand);

  /* ---- VehicleCommand -------------------------------------------------------
   *
   *  Topic: /control/vehicle_cmd
   *
   * ------------------------------------------------------------------------ */
  using VehicleCommand = autoware_vehicle_msgs::msg::VehicleCommand;

  DEFINE_SUBSCRIPTION(VehicleCommand);

#endif  // AUTOWARE_ARCHITECTURE_PROPOSAL

#ifdef AUTOWARE_AUTO
  using GoalPose = geometry_msgs::msg::PoseStamped;
  DEFINE_PUBLISHER(GoalPose);

  using VehicleControlCommand = autoware_auto_msgs::msg::VehicleControlCommand;
  DEFINE_SUBSCRIPTION(VehicleControlCommand);

  using VehicleStateCommand = autoware_auto_msgs::msg::VehicleStateCommand;
  DEFINE_SUBSCRIPTION(VehicleStateCommand);

  using VehicleStateReport = autoware_auto_msgs::msg::VehicleStateReport;
  DEFINE_PUBLISHER(VehicleStateReport);
  decltype(auto) setVehicleStateReport()
  {
    VehicleStateReport report;
    {
      auto current_state_command = getVehicleStateCommand();
      report.stamp = static_cast<Node &>(*this).get_clock()->now();
      report.blinker = current_state_command.blinker;
      report.headlight = current_state_command.headlight;
      report.wiper = current_state_command.wiper;
      report.gear = current_state_command.gear;
      report.mode = current_state_command.mode;
      report.hand_brake = current_state_command.hand_brake;
      report.horn = current_state_command.horn;
    }

    return setVehicleStateReport(report);
  }

  using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
  DEFINE_PUBLISHER(VehicleKinematicState);
  decltype(auto) setVehicleKinematicState(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist)
  {
    VehicleKinematicState kinematic_state;
    {
      kinematic_state.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      kinematic_state.header.frame_id = "map";
      autoware_auto_msgs::msg::TrajectoryPoint state;
      state.x = pose.position.x;
      state.y = pose.position.y;
      state.heading.real = pose.orientation.w;  // from motion_common package of autoware.auto
      state.heading.imag = pose.orientation.z;  // from motion_common package of autoware.auto
      state.longitudinal_velocity_mps = twist.linear.x;
      state.lateral_velocity_mps = twist.linear.y;
      state.acceleration_mps2 = getVehicleControlCommand().long_accel_mps2;
      state.heading_rate_rps = 0.0;  // TODO - what should be the value here?
      state.front_wheel_angle_rad = getVehicleControlCommand().front_wheel_angle_rad;
      kinematic_state.state = state;
    }

    return setVehicleKinematicState(kinematic_state);
  }

  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;
  DEFINE_PUBLISHER(InitialPose);
  decltype(auto) setInitialPose(const geometry_msgs::msg::Pose & pose)
  {
    InitialPose initial_pose;
    {
      initial_pose.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose = pose;
    }

    return setInitialPose(initial_pose);
  }

  using Trajectory = autoware_auto_msgs::msg::Trajectory;
  DEFINE_SUBSCRIPTION(Trajectory);
#endif  // AUTOWARE_AUTO

public:
  explicit MiscellaneousAPI()
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
    : INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(CurrentControlMode, "/vehicle/status/control_mode"),
    INIT_PUBLISHER(CurrentShift, "/vehicle/status/shift"),
    INIT_PUBLISHER(CurrentSteering, "/vehicle/status/steering"),
    INIT_PUBLISHER(CurrentTurnSignal, "/vehicle/status/turn_signal"),
    INIT_PUBLISHER(CurrentTwist, "/vehicle/status/twist"),
    INIT_PUBLISHER(CurrentVelocity, "/vehicle/status/velocity"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    INIT_PUBLISHER(LocalizationTwist, "/localization/twist"),
    INIT_SUBSCRIPTION(Trajectory, "/planning/scenario_planning/trajectory", []() {}),
    INIT_SUBSCRIPTION(TurnSignalCommand, "/control/turn_signal_cmd", []() {}),
    INIT_SUBSCRIPTION(VehicleCommand, "/control/vehicle_cmd", []() {})
#endif  // AUTOWARE_ARCHITECTURE_PROPOSAL
#ifdef AUTOWARE_AUTO
    : INIT_PUBLISHER(GoalPose, "/planning/goal_pose"),
    INIT_PUBLISHER(InitialPose, "/localization/initialpose"),
    INIT_PUBLISHER(VehicleKinematicState, "/vehicle/vehicle_kinematic_state"),
    INIT_PUBLISHER(VehicleStateReport, "/vehicle/state_report"),
    INIT_SUBSCRIPTION(VehicleControlCommand, "/vehicle/vehicle_command", []() {}),
    INIT_SUBSCRIPTION(VehicleStateCommand, "/vehicle/state_command", []() {}),
    INIT_SUBSCRIPTION(Trajectory, "/planning/trajectory", []() {})
#endif  // AUTOWARE_AUTO
  {
  }
};
}  // namespace concealer

#endif  // CONCEALER__MISCELLANEOUS_API_HPP_
