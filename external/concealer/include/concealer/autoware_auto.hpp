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

#ifndef CONCEALER__AUTOWARE_AUTO_HPP_
#define CONCEALER__AUTOWARE_AUTO_HPP_

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <concealer/autoware.hpp>
#include <concealer/define_macro.hpp>

namespace concealer
{
class AutowareAuto : public Autoware
{
  void sendSIGINT() override;

  /// FROM MiscellaneousAPI ///
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
      report.stamp = get_clock()->now();
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
  decltype(auto) setVehicleKinematicState(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist)
  {
    VehicleKinematicState kinematic_state;
    {
      kinematic_state.header.stamp = get_clock()->now();
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
      initial_pose.header.stamp = get_clock()->now();
      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose = pose;
    }

    return setInitialPose(initial_pose);
  }

  using Trajectory = autoware_auto_msgs::msg::Trajectory;
  DEFINE_SUBSCRIPTION(Trajectory);

  autoware_vehicle_msgs::msg::VehicleCommand getVehicleCommand() const override;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public:
  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareAuto(Ts &&... xs)
  : Autoware(std::forward<decltype(xs)>(xs)...),
    INIT_PUBLISHER(GoalPose, "/planning/goal_pose"),
    INIT_SUBSCRIPTION(VehicleControlCommand, "/vehicle/vehicle_command", []() {}),
    INIT_SUBSCRIPTION(VehicleStateCommand, "/vehicle/state_command", []() {}),
    INIT_PUBLISHER(VehicleStateReport, "/vehicle/state_report"),
    INIT_PUBLISHER(VehicleKinematicState, "/vehicle/vehicle_kinematic_state"),
    INIT_PUBLISHER(InitialPose, "/localization/initialpose"),
    INIT_SUBSCRIPTION(Trajectory, "/planning/trajectory", []() {})
  {
    createUpdater();
    waitpid_options = WNOHANG;
  }

  virtual ~AutowareAuto();

  void initialize(const geometry_msgs::msg::Pose &) override;

  void plan(const std::vector<geometry_msgs::msg::PoseStamped> &) override;

  void engage() override;

  void update() override;

  double getAcceleration() const override;

  double getVelocity() const override;

  double getSteeringAngle() const override;

  double getGearSign() const override;

  openscenario_msgs::msg::WaypointsArray getWaypoints() const override;

  double restrictTargetSpeed(double) const override;

  std::string getAutowareStateMessage() const override;
};

}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_AUTO_HPP_
