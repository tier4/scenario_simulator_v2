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
  return getGearCommand().command == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE ? -1.0
                                                                                           : 1.0;
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
}  // namespace concealer
