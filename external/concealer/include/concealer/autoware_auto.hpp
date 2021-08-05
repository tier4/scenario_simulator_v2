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

#include <concealer/autoware.hpp>
#include <concealer/define_macro.hpp>

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>



namespace concealer
{
class AutowareAuto : public Autoware
{
  void sendSIGINT() override {
    std::cout << "AutowareAuto::sendSIGINT" << std::endl;

    sudokill(process_id);
  }

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

  autoware_vehicle_msgs::msg::VehicleCommand getVehicleCommand() const override {
    // gathering information and converting it to autoware_vehicle_msgs::msg::VehicleCommand
    autoware_vehicle_msgs::msg::VehicleCommand vehicle_command;

    auto vehicle_control_command = getVehicleControlCommand();

    vehicle_command.header.stamp = vehicle_control_command.stamp;

    vehicle_command.control.steering_angle = vehicle_control_command.front_wheel_angle_rad;
    vehicle_command.control.velocity = vehicle_control_command.velocity_mps;
    vehicle_command.control.acceleration = vehicle_control_command.long_accel_mps2;

    auto vehicle_state_command = getVehicleStateCommand();

    // handle gear enum remapping
    switch (vehicle_state_command.gear) {
      case autoware_auto_msgs::msg::VehicleStateReport::GEAR_DRIVE:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
        break;
      case autoware_auto_msgs::msg::VehicleStateReport::GEAR_REVERSE:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
        break;
      case autoware_auto_msgs::msg::VehicleStateReport::GEAR_PARK:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
        break;
      case autoware_auto_msgs::msg::VehicleStateReport::GEAR_LOW:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::LOW;
        break;
      case autoware_auto_msgs::msg::VehicleStateReport::GEAR_NEUTRAL:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
        break;
    }

    // these fields are hard-coded because they are not present in AutowareAuto
    vehicle_command.header.frame_id = "";
    vehicle_command.control.steering_angle_velocity = 0.0;
    vehicle_command.emergency = 0;

    return vehicle_command;
  }
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

  virtual ~AutowareAuto() {
    shutdownAutoware();
  }

  void initialize(const geometry_msgs::msg::Pose & initial_pose) override {
    if (not std::exchange(initialize_was_called, true)) {
      task_queue.delay([this, initial_pose]() {
        // TODO: wait for a correct state if necessary once state monitoring is there
        set(initial_pose);
        setInitialPose(initial_pose);
      });
    }
  }

  void plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) override {
    assert(!route.empty());

    if (route.size() > 1) {
      AUTOWARE_WARN_STREAM(
          "AutowareAuto received route consisting of "
              << route.size() << " poses but it does not support checkpoints. Ignoring first "
              << route.size() - 1 << " poses and treating last pose as the goal.");
    }

    task_queue.delay([this, route]() {
      // TODO: replace this sleep with proper state wait logic once state monitoring is there (waitForAutowareStateToBeWaitingForRoute)
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      geometry_msgs::msg::PoseStamped gp;
      gp.pose = route.back().pose;
      gp.pose.position.z = 0.0;
      gp.header.stamp = get_clock()->now();
      gp.header.frame_id = "map";
      setGoalPose(gp);
    });
  }

  void engage() override {
    // TODO: remove this and test
    task_queue.delay(
        // Engage is not implemented in Autoware.Auto
        [this]() {});
  }


  void update() override {
    setTransform(current_pose);
    setVehicleKinematicState(current_pose, current_twist);
    setVehicleStateReport();
  }
  double getAcceleration() const override {
      return getVehicleControlCommand().long_accel_mps2;
  }

  double getVelocity() const override {
    return getVehicleControlCommand().velocity_mps;
  }

  double getSteeringAngle() const override {
    return getVehicleControlCommand().front_wheel_angle_rad;
  }

  double getGearSign() const override {
    return getVehicleStateCommand().gear == autoware_auto_msgs::msg::VehicleStateReport::GEAR_REVERSE ? -1.0 : +1.0;
  }

  openscenario_msgs::msg::WaypointsArray getWaypoints() const override {
    openscenario_msgs::msg::WaypointsArray waypoints;

    for (const auto & point : getTrajectory().points) {
      geometry_msgs::msg::Point waypoint;
      waypoint.x = point.x;
      waypoint.y = point.y;
      waypoint.z = 0;
      waypoints.waypoints.push_back(waypoint);
    }

    return waypoints;
  }

  double restrictTargetSpeed(double) const override {
    // non-zero initial speed prevents behavioral planner from planning so it must be restricted to 0
    return 0;
  }

  std::string getAutowareStateMessage() const override {
    // TODO: implement when state monitoring is there
    return {};
  }

};

}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_AUTO_HPP_
