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

auto AutowareUniverse::update() -> void
{
  setControlModeReport([this]() {
    ControlModeReport message;
    message.mode = ControlModeReport::AUTONOMOUS;
    return message;
  }());

  setAcceleration([this]() {
    Acceleration message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "/base_link";
    message.accel.accel = current_acceleration;
    message.accel.covariance.at(6 * 0 + 0) = 0.001;  // linear x
    message.accel.covariance.at(6 * 1 + 1) = 0.001;  // linear y
    message.accel.covariance.at(6 * 2 + 2) = 0.001;  // linear z
    message.accel.covariance.at(6 * 3 + 3) = 0.001;  // angular x
    message.accel.covariance.at(6 * 4 + 4) = 0.001;  // angular y
    message.accel.covariance.at(6 * 5 + 5) = 0.001;  // angular z
    return message;
  }());

  setGearReport([this]() {
    GearReport message;
    message.stamp = get_clock()->now();
    message.report = getGearCommand().command;
    return message;
  }());

  setSteeringReport([this]() {
    SteeringReport message;
    message.stamp = get_clock()->now();
    message.steering_tire_angle = getSteeringAngle();
    return message;
  }());

  setOdometry([this]() {
    Odometry message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "map";
    message.pose.pose = current_pose;
    message.pose.covariance = {};
    message.twist.twist = current_twist;
    return message;
  }());

  setVelocityReport([this]() {
    VelocityReport message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "base_link";
    message.longitudinal_velocity = current_twist.linear.x;
    message.lateral_velocity = current_twist.linear.y;
    message.heading_rate = current_twist.angular.z;
    return message;
  }());

  setTurnIndicatorsReport([this]() {
    TurnIndicatorsReport message;
    message.stamp = get_clock()->now();
    message.report = getTurnIndicatorsCommand().command;
    return message;
  }());

  setTransform(current_pose);
}

auto AutowareUniverse::getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand
{
  return getGearCommandImpl();
}

auto AutowareUniverse::getGearSign() const -> double
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  return getGearCommand().command == GearCommand::REVERSE or
             getGearCommand().command == GearCommand::REVERSE_2
           ? -1.0
           : 1.0;
}

auto AutowareUniverse::getVehicleCommand() const -> std::tuple<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  autoware_auto_vehicle_msgs::msg::GearCommand>
{
  return std::make_tuple(getAckermannControlCommand(), getGearCommand());
}
}  // namespace concealer