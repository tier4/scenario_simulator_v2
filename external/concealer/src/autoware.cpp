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

#include <concealer/autoware.hpp>

namespace concealer
{
Autoware::Autoware()
: rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
  current_acceleration(geometry_msgs::msg::Accel()),
  current_twist(geometry_msgs::msg::Twist()),
  current_pose(geometry_msgs::msg::Pose())
{
}

auto Autoware::getGearCommand() const -> tier4_external_api_msgs::msg::GearShiftStamped
{
  static auto gear_command = []() {
    tier4_external_api_msgs::msg::GearShiftStamped gear_command;
    gear_command.gear_shift.data = tier4_external_api_msgs::msg::GearShift::DRIVE;
    return gear_command;
  }();
  gear_command.stamp = now();
  return gear_command;
}

auto Autoware::set(const geometry_msgs::msg::Accel & acceleration) -> void
{
  current_acceleration.store(acceleration);
}

auto Autoware::set(const geometry_msgs::msg::Twist & twist) -> void { current_twist.store(twist); }

auto Autoware::set(const geometry_msgs::msg::Pose & pose) -> void { current_pose.store(pose); }

auto Autoware::getTurnIndicatorsCommand() const
  -> tier4_external_api_msgs::msg::TurnSignalStamped
{
  static auto turn_indicators_command = []() {
    tier4_external_api_msgs::msg::TurnSignalStamped turn_indicators_command;
    turn_indicators_command.turn_signal.data =
      tier4_external_api_msgs::msg::TurnSignal::NONE;
    return turn_indicators_command;
  }();
  turn_indicators_command.stamp = now();
  return turn_indicators_command;
}

auto Autoware::rethrow() -> void {}
}  // namespace concealer
