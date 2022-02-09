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

#include <concealer/autoware_auto.hpp>

namespace concealer
{
AutowareAuto::~AutowareAuto() { shutdownAutoware(); }

void AutowareAuto::initialize(const geometry_msgs::msg::Pose & initial_pose)
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      // TODO: wait for a correct state if necessary once state monitoring is there
      set(initial_pose);
      setInitialPose(initial_pose);
    });
  }
}

void AutowareAuto::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route)
{
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

void AutowareAuto::engage()
{
  task_queue.delay(
    // Engage is not implemented in Autoware.Auto
    [this]() {});
}

void AutowareAuto::update()
{
  setTransform(current_pose);
  setVehicleKinematicState(current_pose, current_twist);
  setVehicleStateReport();
}

double AutowareAuto::getAcceleration() const { return getVehicleControlCommand().long_accel_mps2; }

double AutowareAuto::getVelocity() const { return getVehicleControlCommand().velocity_mps; }

double AutowareAuto::getSteeringAngle() const
{
  return getVehicleControlCommand().front_wheel_angle_rad;
}

double AutowareAuto::getGearSign() const
{
  using AUTOWARE_AUTO_VEHICLE_MSGS::msg::VehicleStateReport;
  return getVehicleStateCommand().gear == VehicleStateReport::GEAR_REVERSE ? -1.0 : +1.0;
}

auto AutowareAuto::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    geometry_msgs::msg::Point waypoint;
    waypoint.x = point.pose.position.x;
    waypoint.y = point.pose.position.y;
    waypoint.z = 0;
    waypoints.waypoints.push_back(waypoint);
  }

  return waypoints;
}

double AutowareAuto::restrictTargetSpeed(double) const
{
  // non-zero initial speed prevents behavioral planner from planning so it must be restricted to 0
  return 0;
}

std::string AutowareAuto::getAutowareStateMessage() const
{
  // TODO: implement when state monitoring is there
  return {};
}

auto AutowareAuto::getVehicleCommand() const -> std::tuple<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  autoware_auto_vehicle_msgs::msg::GearCommand>
{
  return std::make_tuple(
    autoware_auto_control_msgs::msg::AckermannControlCommand(),
    autoware_auto_vehicle_msgs::msg::GearCommand());
}

void AutowareAuto::sendSIGINT() { sudokill(process_id); }
}  // namespace concealer
