// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <joy_to_vehicle_cmd/joy_to_vehicle_cmd_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace joy_to_vehicle_cmd
{
JoyToVehicleCommandComponent::JoyToVehicleCommandComponent(const rclcpp::NodeOptions & options)
: Node("joy_to_vehicle_cmd", options)
{
  declare_parameter("velocity_axes_index", 1);
  get_parameter("velocity_axes_index", velocity_axes_index_);
  declare_parameter("angluar_axes_index", 3);
  get_parameter("angluar_axes_index", angluar_axes_index_);
  joy_sub_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1,
    std::bind(&JoyToVehicleCommandComponent::JoyCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>(
    "input/vehicle_control_command", 1);
}

void JoyToVehicleCommandComponent::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  current_cmd_.front_wheel_angle_rad = msg->axes[angluar_axes_index_];
  current_cmd_.velocity_mps = msg->axes[velocity_axes_index_] * 5;
  cmd_pub_->publish(current_cmd_);
}
}  // namespace joy_to_vehicle_cmd

RCLCPP_COMPONENTS_REGISTER_NODE(joy_to_vehicle_cmd::JoyToVehicleCommandComponent)
