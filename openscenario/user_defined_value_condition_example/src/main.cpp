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

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter_msgs/msg/parameter_declaration.hpp>
#include <openscenario_interpreter_msgs/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>

int main(const int argc, char const * const * const argv)
{
  using openscenario_interpreter_msgs::msg::ParameterDeclaration;
  using openscenario_interpreter_msgs::msg::ParameterType;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("this_node_name");

  auto publisher = node->create_publisher<ParameterDeclaration>(
    "/simulation/this_node_name", rclcpp::QoS(1).reliable());

  auto make_message = [&, count = 0]() mutable  //
  {
    ParameterDeclaration message;
    {
      message.name = "Currently ParameterDeclaration::name will be ignored.";
      message.parameter_type = ParameterType::UNSIGNED_INT;
      message.value = boost::lexical_cast<decltype(message.value)>(++count);
    }

    return message;
  };

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100), [&]() { publisher->publish(make_message()); });

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();

  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
