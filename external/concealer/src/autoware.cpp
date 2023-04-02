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
: rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false))
{
}

auto Autoware::spinSome() -> void
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(get_node_base_interface());
  }
}

auto Autoware::getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand
{
  static auto gear_command = []() {
    autoware_auto_vehicle_msgs::msg::GearCommand gear_command;
    gear_command.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    return gear_command;
  }();
  gear_command.stamp = now();
  return gear_command;
}

auto Autoware::set(const geometry_msgs::msg::Accel & acceleration)
  -> const geometry_msgs::msg::Accel &
{
  return current_acceleration = acceleration;
}

auto Autoware::set(const geometry_msgs::msg::Twist & twist) -> const geometry_msgs::msg::Twist &
{
  return current_twist = twist;
}

auto Autoware::set(const geometry_msgs::msg::Pose & pose) -> const geometry_msgs::msg::Pose &
{
  return current_pose = pose;
}

auto Autoware::getTurnIndicatorsCommand() const
  -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand
{
  static auto turn_indicators_command = []() {
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicators_command;
    turn_indicators_command.command =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
    return turn_indicators_command;
  }();
  turn_indicators_command.stamp = now();
  return turn_indicators_command;
}

auto Autoware::rethrow() -> void{};
}  // namespace concealer
