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

#ifndef CONCEALER__AUTOWARE_HPP_
#define CONCEALER__AUTOWARE_HPP_

#include <atomic>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
/**
  * Provides an abstraction to communicate with Autoware in order to:
  * - receive vehicle commands to simulate vehicle kinematics
  * - provide vehicle state reports on an appropriate topics
  * NOTE: This class is intended to be move to simple_sensor_simulator
  */
class Autoware : public rclcpp::Node, public ContinuousTransformBroadcaster<Autoware>
{
protected:
  std::atomic<geometry_msgs::msg::Accel> current_acceleration;

  std::atomic<geometry_msgs::msg::Twist> current_twist;

  std::atomic<geometry_msgs::msg::Pose> current_pose;

public:
  CONCEALER_PUBLIC explicit Autoware();

  virtual auto getAcceleration() const -> double = 0;

  virtual auto getGearCommand() const -> autoware_vehicle_msgs::msg::GearCommand;

  virtual auto getSteeringAngle() const -> double = 0;

  virtual auto getVelocity() const -> double = 0;

  // returns -1.0 when gear is reverse and 1.0 otherwise
  virtual auto getGearSign() const -> double = 0;

  virtual auto getTurnIndicatorsCommand() const
    -> autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

  virtual auto getVehicleCommand() const
    -> std::tuple<autoware_control_msgs::msg::Control, autoware_vehicle_msgs::msg::GearCommand> = 0;

  virtual auto getRouteLanelets() const -> std::vector<std::int64_t> = 0;

  auto set(const geometry_msgs::msg::Accel &) -> void;

  auto set(const geometry_msgs::msg::Twist &) -> void;

  auto set(const geometry_msgs::msg::Pose &) -> void;

  virtual auto rethrow() -> void;

  virtual auto setManualMode() -> void = 0;

  virtual auto setAutonomousMode() -> void = 0;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_HPP_
