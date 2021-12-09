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

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO

#include <boost/range/adaptor/sliced.hpp>
#include <concealer/autoware_universe.hpp>

namespace concealer
{
AutowareUniverse::~AutowareUniverse() { shutdownAutoware(); }

auto AutowareUniverse::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      set(initial_pose);
      waitForAutowareStateToBeInitializingVehicle();
      waitForAutowareStateToBeWaitingForRoute([&]() {
        InitialPose initial_pose;
        {
          initial_pose.header.stamp = get_clock()->now();
          initial_pose.header.frame_id = "map";
          initial_pose.pose.pose = current_pose;
        }
        return setInitialPose(initial_pose);
      });
    });
  }
}

auto AutowareUniverse::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) -> void
{
  assert(not route.empty());

  task_queue.delay([this, route] {
    waitForAutowareStateToBeWaitingForRoute();  // NOTE: This is assertion.
    setGoalPose(route.back());
    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      setCheckpoint(each);
    }
    waitForAutowareStateToBePlanning();
    waitForAutowareStateToBeWaitingForEngage();  // NOTE: Autoware.IV 0.11.1 waits about 3 sec from the completion of Planning until the transition to WaitingForEngage.
  });
}

auto AutowareUniverse::engage() -> void
{
  task_queue.delay([this]() {
    waitForAutowareStateToBeDriving([this]() {
      AutowareEngage message;
      {
        message.stamp = get_clock()->now();
        message.engage = true;
      }
      return setAutowareEngage(message);
    });
  });
}

auto AutowareUniverse::update() -> void
{
  CurrentControlMode current_control_mode;
  {
    current_control_mode.mode = CurrentControlMode::AUTONOMOUS;
  }
  setCurrentControlMode(current_control_mode);

  CurrentShift current_shift;
  {
    using autoware_auto_vehicle_msgs::msg::GearReport;
    current_shift.stamp = get_clock()->now();
    current_shift.report = current_twist.linear.x >= 0 ? GearReport::DRIVE : GearReport::REVERSE;
  }
  setCurrentShift(current_shift);

  CurrentSteering current_steering;
  {
    current_steering.stamp = get_clock()->now();
    current_steering.steering_tire_angle = getSteeringAngle();
  }
  setCurrentSteering(current_steering);

  CurrentVelocity current_velocity;
  {
    current_velocity.header.stamp = get_clock()->now();
    current_velocity.header.frame_id = "base_link";
    current_velocity.longitudinal_velocity = current_twist.linear.x;
    current_velocity.lateral_velocity = current_twist.linear.y;
    current_velocity.heading_rate = current_twist.angular.z;
  }
  setCurrentVelocity(current_velocity);

  LocalizationOdometry localization_odometry;
  {
    localization_odometry.header.stamp = get_clock()->now();
    localization_odometry.header.frame_id = "map";
    localization_odometry.pose.pose = current_pose;
    localization_odometry.pose.covariance = {};
    localization_odometry.twist.twist = current_twist;
  }
  setLocalizationOdometry(localization_odometry);

  VehicleVelocity vehicle_velocity;
  {
    vehicle_velocity.stamp = get_clock()->now();
    vehicle_velocity.max_velocity = current_upper_bound_speed;
  }
  setVehicleVelocity(vehicle_velocity);

  setTransform(current_pose);
}

auto AutowareUniverse::getAcceleration() const -> double
{
  return getAckermannControlCommand().longitudinal.acceleration;
}

auto AutowareUniverse::getVelocity() const -> double
{
  return getAckermannControlCommand().longitudinal.speed;
}

auto AutowareUniverse::getSteeringAngle() const -> double
{
  return getAckermannControlCommand().lateral.steering_tire_angle;
}

auto AutowareUniverse::getGearSign() const -> double
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  return getGearCommand().command == GearCommand::REVERSE ? -1.0 : 1.0;
}

auto AutowareUniverse::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto AutowareUniverse::restrictTargetSpeed(double value) const -> double
{
  // no restrictions here
  return value;
}

auto AutowareUniverse::getAutowareStateMessage() const -> std::string
{
  return getAutowareStatus().autoware_state;
}

auto AutowareUniverse::sendSIGINT() -> void  //
{
  ::kill(process_id, SIGINT);
}

auto AutowareUniverse::isReady() noexcept -> bool
{
  return is_ready or (is_ready = isWaitingForRoute());
}

auto AutowareUniverse::checkAutowareState() -> void
{
  if (isReady() and isEmergency()) {
    // throw common::AutowareError("Autoware is in emergency state now");
  }
}

auto AutowareUniverse::getVehicleCommand() const -> autoware_vehicle_msgs::msg::VehicleCommand
{
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_command;
  {
    auto ackermann_control_command = getAckermannControlCommand();

    vehicle_command.header.stamp = ackermann_control_command.stamp;
    vehicle_command.control.steering_angle = ackermann_control_command.lateral.steering_tire_angle;
    vehicle_command.control.velocity = ackermann_control_command.longitudinal.speed;
    vehicle_command.control.acceleration = ackermann_control_command.longitudinal.acceleration;

    auto gear_command = getGearCommand();

    switch (gear_command.command) {
      case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::PARK:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::LOW:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::LOW;
        break;
      case 0:
        vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
        break;
    }

    // these fields are hard-coded because they are not present in AutowareAuto
    vehicle_command.header.frame_id = "";
    vehicle_command.control.steering_angle_velocity = 0.0;
    vehicle_command.emergency = 0;
  }

  return vehicle_command;
}
}  // namespace concealer

#else  // ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO

#include <concealer/autoware_universe.hpp>

namespace concealer
{
AutowareUniverse::~AutowareUniverse() { shutdownAutoware(); }

auto AutowareUniverse::engage() -> void {}

auto AutowareUniverse::getAcceleration() const -> double { return {}; }

auto AutowareUniverse::getAutowareStateMessage() const -> std::string { return {}; }

auto AutowareUniverse::getGearSign() const -> double { return 1.0; }

auto AutowareUniverse::getSteeringAngle() const -> double { return {}; }

auto AutowareUniverse::getVehicleCommand() const -> autoware_vehicle_msgs::msg::VehicleCommand
{
  return {};
}

auto AutowareUniverse::getVelocity() const -> double { return {}; }

auto AutowareUniverse::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  return {};
}

auto AutowareUniverse::initialize(const geometry_msgs::msg::Pose &) -> void {}

auto AutowareUniverse::plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void {}

auto AutowareUniverse::update() -> void {}

auto AutowareUniverse::restrictTargetSpeed(double value) const -> double { return value; }

auto AutowareUniverse::sendSIGINT() -> void { ::kill(process_id, SIGINT); }
}  // namespace concealer

#endif  // SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
