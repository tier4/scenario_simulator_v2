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

#include <concealer/autoware_architecture_proposal.hpp>

namespace concealer
{
AutowareArchitectureProposal::~AutowareArchitectureProposal() { shutdownAutoware(); }

auto AutowareArchitectureProposal::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      set(initial_pose);
      waitForAutowareStateToBeInitializingVehicle();
      waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
    });
  }
}

auto AutowareArchitectureProposal::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route)
  -> void
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

auto AutowareArchitectureProposal::engage() -> void
{
  task_queue.delay(
    [this]() { waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); }); });
}

auto AutowareArchitectureProposal::update() -> void
{
  setCurrentControlMode();
  setCurrentShift(current_twist);
  setCurrentSteering(current_twist);
  setCurrentTwist(current_twist);
  setCurrentVelocity(current_twist);
  setLocalizationPose(current_pose);
  setLocalizationTwist(current_twist);
  setTransform(current_pose);
}

auto AutowareArchitectureProposal::getAcceleration() const -> double
{
  return getVehicleCommand().control.acceleration;
}

auto AutowareArchitectureProposal::getVelocity() const -> double
{
  return getVehicleCommand().control.velocity;
}

auto AutowareArchitectureProposal::getSteeringAngle() const -> double
{
  return getVehicleCommand().control.steering_angle;
}

auto AutowareArchitectureProposal::getGearSign() const -> double
{
  return getVehicleCommand().shift.data == autoware_vehicle_msgs::msg::Shift::REVERSE ? -1.0 : 1.0;
}

auto AutowareArchitectureProposal::getWaypoints() const -> openscenario_msgs::msg::WaypointsArray
{
  openscenario_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto AutowareArchitectureProposal::restrictTargetSpeed(double value) const -> double
{
  // no restrictions here
  return value;
}

auto AutowareArchitectureProposal::getAutowareStateMessage() const -> std::string
{
  return getAutowareStatus().autoware_state;
}

auto AutowareArchitectureProposal::sendSIGINT() -> void  //
{
  ::kill(process_id, SIGINT);
}

auto AutowareArchitectureProposal::isReady() noexcept -> bool
{
  return is_ready or (is_ready = isWaitingForRoute());
}

auto AutowareArchitectureProposal::isNotReady() noexcept -> bool  //
{
  return not isReady();
}

auto AutowareArchitectureProposal::checkAutowareState() -> void
{
  if (isReady() and isEmergency()) {
    // throw common::AutowareError("Autoware is in emergency state now");
  }
}

auto AutowareArchitectureProposal::setUpperBoundSpeed(double value) -> double
{
  VehicleVelocity vehicle_velocity;
  {
    vehicle_velocity.stamp = get_clock()->now();
    vehicle_velocity.max_velocity = value;
  }

  setVehicleVelocity(vehicle_velocity);

  return value;
}
}  // namespace concealer
