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

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <boost/lexical_cast.hpp>
#include <openscenario_msgs/msg/parameter_declaration.hpp>
#include <openscenario_msgs/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>

int main(const int argc, char const * const * const argv)
{
  using openscenario_msgs::msg::ParameterDeclaration;
  using openscenario_msgs::msg::ParameterType;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("count_up");

  autoware_api_msgs::msg::AwapiAutowareStatus status;

  auto subscription = node->create_subscription<autoware_api_msgs::msg::AwapiAutowareStatus>(
    "/awapi/autoware/get/status", rclcpp::QoS(1).reliable(),
    [&](const autoware_api_msgs::msg::AwapiAutowareStatus::SharedPtr message) {
      status = *message;
    });

  auto publisher =
    node->create_publisher<ParameterDeclaration>("/timeout", rclcpp::QoS(1).reliable());

  auto make_message = [&](const auto & status) mutable  //
  {
    static auto duration_since_autoware_engaged = std::chrono::high_resolution_clock::now();

    if (status.autoware_state != autoware_system_msgs::msg::AutowareState::DRIVING) {
      duration_since_autoware_engaged = std::chrono::high_resolution_clock::now();
    }

    ParameterDeclaration message;
    {
      message.name = "";
      message.parameter_type.data = ParameterType::BOOLEAN;
      message.value =
        (10 < std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - duration_since_autoware_engaged)
                .count())
          ? "true"
          : "false";
    }

    std::cout << "message.value = " << message.value << std::endl;

    return message;
  };

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100), [&]() { publisher->publish(make_message(status)); });

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();

  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
