// Copyright 2015 TIER IV, Inc. All rights reserved.
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
AutowareUniverse::AutowareUniverse()
: getAckermannControlCommand("/control/command/control_cmd", *this),
  // getControlCommand("/api/external/set/command/local/control", *this),
  getGearCommandImpl("/api/external/set/command/local/shift", *this),
  getTurnIndicatorsCommand("/api/external/set/command/local/turn_signal", *this),
  getPathWithLaneId(
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", *this),
  setAcceleration("/localization/acceleration", *this),
  setOdometry("/localization/kinematic_state", *this),
  setVehicleStatus("/api/external/get/vehicle/status", *this),
  setControlModeReport("/vehicle/status/control_mode", *this),
  // Autoware.Universe requires localization topics to send data at 50Hz
  localization_update_timer(rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(20), [this]() { updateLocalization(); })),
  // Autoware.Universe requires vehicle state topics to send data at 30Hz
  vehicle_state_update_timer(rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(33), [this]() { updateVehicleState(); })),
  localization_and_vehicle_state_update_thread(std::thread([this]() {
    try {
      while (rclcpp::ok() and not is_stop_requested.load()) {
        rclcpp::spin_some(get_node_base_interface());
      }
    } catch (...) {
      thrown = std::current_exception();
      is_thrown.store(true);
    }
  }))
{
}

AutowareUniverse::~AutowareUniverse() { stopAndJoin(); }

auto AutowareUniverse::rethrow() -> void
{
  if (is_thrown.load()) {
    throw thrown;
  }
}

auto AutowareUniverse::stopAndJoin() -> void
{
  is_stop_requested.store(true);
  localization_and_vehicle_state_update_thread.join();
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

auto AutowareUniverse::updateLocalization() -> void
{
  setAcceleration([this]() {
    geometry_msgs::msg::AccelWithCovarianceStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "/base_link";
    message.accel.accel = current_acceleration.load();
    message.accel.covariance.at(6 * 0 + 0) = 0.001;  // linear x
    message.accel.covariance.at(6 * 1 + 1) = 0.001;  // linear y
    message.accel.covariance.at(6 * 2 + 2) = 0.001;  // linear z
    message.accel.covariance.at(6 * 3 + 3) = 0.001;  // angular x
    message.accel.covariance.at(6 * 4 + 4) = 0.001;  // angular y
    message.accel.covariance.at(6 * 5 + 5) = 0.001;  // angular z
    return message;
  }());

  setOdometry([this]() {
    nav_msgs::msg::Odometry message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "map";
    message.pose.pose = current_pose.load();
    message.pose.covariance = {};
    message.twist.twist = current_twist.load();
    return message;
  }());

  setTransform(current_pose.load());
}

auto AutowareUniverse::updateVehicleState() -> void
{
  setControlModeReport([this]() {
    autoware_auto_vehicle_msgs::msg::ControlModeReport message;
    message.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    return message;
  }());

  setVehicleStatus([this]() {
    tier4_external_api_msgs::msg::VehicleStatusStamped message;
    message.stamp = get_clock()->now();
    const auto twist = current_twist.load();
    message.status.twist = twist;
    message.status.steering.data = getSteeringAngle();
    message.status.turn_signal.data = getTurnIndicatorsCommand().turn_signal.data;
    message.status.gear_shift.data = getTurnIndicatorsCommand().turn_signal.data;
    message.status.gear_shift.data = getGearCommand().gear_shift.data;
    return message;
  }());
}

auto AutowareUniverse::getGearCommand() const -> tier4_external_api_msgs::msg::GearShiftStamped
{
  return getGearCommandImpl();
}

auto AutowareUniverse::getGearSign() const -> double
{
  using tier4_external_api_msgs::msg::GearShift;
  return getGearCommand().gear_shift.data == GearShift::REVERSE
           ? -1.0
           : 1.0;
}

auto AutowareUniverse::getVehicleCommand() const -> std::tuple<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  tier4_external_api_msgs::msg::GearShiftStamped>
{
  return std::make_tuple(getAckermannControlCommand(), getGearCommand());
}

auto AutowareUniverse::getRouteLanelets() const -> std::vector<std::int64_t>
{
  std::vector<std::int64_t> ids{};
  for (const auto & point : getPathWithLaneId().points) {
    std::copy(point.lane_ids.begin(), point.lane_ids.end(), std::back_inserter(ids));
  }
  return ids;
}
}  // namespace concealer
