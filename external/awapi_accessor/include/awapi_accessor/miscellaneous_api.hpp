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

#include <awapi_accessor/conversion.hpp>
#include <awapi_accessor/define_macro.hpp>

#if defined AUTOWARE_IV
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#elif defined AUTOWARE_AUTO
// TODO (someone else)
#endif

namespace awapi
{
template <typename Node>
class MiscellaneousAPI
{
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

  /* ---- CurrentPose ----------------------------------------------------------
   *
   *  Topic: /current_pose
   *
   * ------------------------------------------------------------------------ */
  using CurrentPose = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(CurrentPose);

  decltype(auto) setCurrentPose(const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    {
      current_pose.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      current_pose.header.frame_id = "map";
      current_pose.pose = pose;
    }

    return setCurrentPose(current_pose);
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

  /* ---- InitialTwist ---------------------------------------------------------
   *
   *  Set initial velocity of Autoware.
   *
   *  Topic: /initialtwist
   *
   * ------------------------------------------------------------------------ */
  using InitialTwist = geometry_msgs::msg::TwistStamped;

  DEFINE_PUBLISHER(InitialTwist);

  decltype(auto) setInitialTwist(
    const geometry_msgs::msg::Twist & twist = geometry_msgs::msg::Twist())
  {
    InitialTwist initial_twist;
    {
      initial_twist.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      initial_twist.header.frame_id = "map";
      initial_twist.twist = twist;
    }

    AWAPI_CURRENT_VALUE_OF(VehicleCommand).control.velocity = initial_twist.twist.linear.x;

    return setInitialTwist(initial_twist);
  }

  decltype(auto) setInitialTwist(const double linear_x, const double angular_z = 0)
  {
    geometry_msgs::msg::Twist twist;
    {
      twist.linear.x = linear_x;
      twist.angular.z = angular_z;
    }

    return setInitialTwist(twist);
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
    InitialTwist localization_twist;
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

  /* ---- InitialTwist ---------------------------------------------------------
   *
   *  Set initial velocity of Autoware.
   *
   *  Topic: /initialtwist
   *
   * ------------------------------------------------------------------------ */
  using VehicleCommand = autoware_vehicle_msgs::msg::VehicleCommand;

  DEFINE_SUBSCRIPTION(VehicleCommand);

public:
  explicit MiscellaneousAPI()
  : INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(CurrentControlMode, "/vehicle/status/control_mode"),
    INIT_PUBLISHER(CurrentPose, "/current_pose"),
    INIT_PUBLISHER(CurrentShift, "/vehicle/status/shift"),
    INIT_PUBLISHER(CurrentSteering, "/vehicle/status/steering"),
    INIT_PUBLISHER(CurrentTurnSignal, "/vehicle/status/turn_signal"),
    INIT_PUBLISHER(CurrentTwist, "/vehicle/status/twist"),
    INIT_PUBLISHER(CurrentVelocity, "/vehicle/status/velocity"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    INIT_PUBLISHER(InitialTwist, "/initialtwist"),
    INIT_PUBLISHER(LocalizationTwist, "/localization/twist"),
    INIT_SUBSCRIPTION(Trajectory, "/planning/scenario_planning/trajectory", []() {}),
    INIT_SUBSCRIPTION(TurnSignalCommand, "/control/turn_signal_cmd", []() {}),
    INIT_SUBSCRIPTION(VehicleCommand, "/control/vehicle_cmd", []() {})
  {
  }
};
}  // namespace awapi
