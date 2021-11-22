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
      waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
    });
  }
}

auto AutowareUniverse::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) -> void
{
  assert(!route.empty());

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
  task_queue.delay(
    [this]() { waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); }); });
}

auto AutowareUniverse::update() -> void
{
  setCurrentControlMode();
  setCurrentShift(current_twist);
  setCurrentSteering();
  setCurrentVelocity(current_twist);
  setLocalizationOdometry(current_pose, current_twist);
  setTransform(current_pose);
  setVehicleVelocity(current_upper_bound_speed);
}

auto AutowareUniverse::getAcceleration() const -> double
{
  return getVehicleCommand().control.acceleration;
}

auto AutowareUniverse::getVelocity() const -> double
{
  return getVehicleCommand().control.velocity;
}

auto AutowareUniverse::getSteeringAngle() const -> double
{
  return getVehicleCommand().control.steering_angle;
}

auto AutowareUniverse::getGearSign() const -> double
{
  return getVehicleCommand().shift.data == autoware_vehicle_msgs::msg::Shift::REVERSE ? -1.0 : 1.0;
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

auto AutowareUniverse::isNotReady() noexcept -> bool  //
{
  return not isReady();
}

auto AutowareUniverse::checkAutowareState() -> void
{
  if (isReady() and isEmergency()) {
    // throw common::AutowareError("Autoware is in emergency state now");
  }
}

autoware_vehicle_msgs::msg::VehicleCommand AutowareUniverse::getVehicleCommand() const
{
  // gathering information and converting it to autoware_vehicle_msgs::msg::VehicleCommand
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_command;

  auto vehicle_control_command = getAckermannControlCommand();

  vehicle_command.header.stamp = vehicle_control_command.stamp;

  vehicle_command.control.steering_angle = vehicle_control_command.lateral.steering_tire_angle;
  vehicle_command.control.steering_angle_velocity =
    vehicle_control_command.lateral.steering_tire_rotation_rate;
  vehicle_command.control.velocity = vehicle_control_command.longitudinal.speed;
  vehicle_command.control.acceleration = vehicle_control_command.longitudinal.acceleration;

  //auto vehicle_state_command = getVehicleStateCommand();
  auto turn_indicators_command = getGearCommand();

  using autoware_auto_vehicle_msgs::msg::GearCommand;

  // handle gear enum remapping
  switch (turn_indicators_command.command) {
    case GearCommand::DRIVE:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
      break;
    case GearCommand::REVERSE:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
      break;
    case GearCommand::PARK:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
      break;
    case GearCommand::LOW:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::LOW;
      break;
  }

  // these fields are hard-coded because they are not present in AutowareAuto
  vehicle_command.header.frame_id = "";
  vehicle_command.emergency = 0;

  return vehicle_command;
}
}  // namespace concealer
