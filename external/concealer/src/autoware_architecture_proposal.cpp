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

  AutowareArchitectureProposal::~AutowareArchitectureProposal() {
    shutdownAutoware();
  }

  void AutowareArchitectureProposal::initialize(const geometry_msgs::msg::Pose & initial_pose) {
    if (not std::exchange(initialize_was_called, true)) {
      task_queue.delay([this, initial_pose]() {
        set(initial_pose);
        waitForAutowareStateToBeInitializingVehicle();
        waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
      });
    }
  }

  void AutowareArchitectureProposal::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) {
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

  void AutowareArchitectureProposal::engage() {
    task_queue.delay(
        [this]() { waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); }); });
  }

  void AutowareArchitectureProposal::update() {
    setCurrentControlMode();
    setCurrentShift(current_twist);
    setCurrentSteering(current_twist);
    setCurrentTwist(current_twist);
    setCurrentVelocity(current_twist);
    setLocalizationPose(current_pose);
    setLocalizationTwist(current_twist);
    setTransform(current_pose);
  }

  double AutowareArchitectureProposal::getAcceleration() const {
      return getVehicleCommand().control.acceleration;
  }

  double AutowareArchitectureProposal::getVelocity() const {
    return getVehicleCommand().control.velocity;
  }

  double AutowareArchitectureProposal::getSteeringAngle() const {
    return getVehicleCommand().control.steering_angle;
  }

  double AutowareArchitectureProposal::getGearSign() const {
    return getVehicleCommand().shift.data == autoware_vehicle_msgs::msg::Shift::REVERSE ? -1.0 : 1.0;
  }

  openscenario_msgs::msg::WaypointsArray AutowareArchitectureProposal::getWaypoints() const {
    openscenario_msgs::msg::WaypointsArray waypoints;

    for (const auto & point : getTrajectory().points) {
      waypoints.waypoints.emplace_back(point.pose.position);
    }

    return waypoints;
  }

  double AutowareArchitectureProposal::restrictTargetSpeed(double value) const {
    // no restrictions here
    return value;
  }

  std::string AutowareArchitectureProposal::getAutowareStateMessage() const {
    return getAutowareStatus().autoware_state;
  }

  void AutowareArchitectureProposal::sendSIGINT() {
    ::kill(process_id, SIGINT);
  }

  bool AutowareArchitectureProposal::isReady() noexcept {
    return is_ready or (is_ready = isWaitingForRoute());
  }

  bool AutowareArchitectureProposal::isNotReady() noexcept {
    return not isReady();
  }

  void AutowareArchitectureProposal::checkAutowareState() {
    if (isReady() and isEmergency()) {
      // throw common::AutowareError("Autoware is in emergency state now");
    }
  }

}  // namespace concealer
