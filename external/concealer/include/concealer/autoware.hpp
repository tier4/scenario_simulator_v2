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

#include <concealer/utility/visibility.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace concealer {

class Autoware : public rclcpp::Node {

protected:
  geometry_msgs::msg::Accel current_acceleration;

public:

  CONCEALER_PUBLIC explicit Autoware()
      : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)) {

  }

  virtual auto getAcceleration() const -> double = 0;

  virtual auto getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand;

  virtual auto getSteeringAngle() const -> double = 0;

  virtual auto getVelocity() const -> double = 0;

  // returns -1.0 when gear is reverse and 1.0 otherwise
  virtual auto getGearSign() const -> double = 0;

  virtual auto getVehicleCommand() const -> std::tuple<
      autoware_auto_control_msgs::msg::AckermannControlCommand,
      autoware_auto_vehicle_msgs::msg::GearCommand> = 0;

  /*   */ auto set(const geometry_msgs::msg::Accel &) -> const geometry_msgs::msg::Accel &;

  virtual auto update() -> void = 0;

  void spinSome() {
    if (rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
    }
  }
};
}

#endif //CONCEALER__AUTOWARE_HPP
