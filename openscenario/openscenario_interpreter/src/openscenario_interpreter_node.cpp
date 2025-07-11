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

#include <glog/logging.h>

#include <cstdlib>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <status_monitor/status_monitor.hpp>

int main(const int argc, char const * const * const argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<openscenario_interpreter::Interpreter>(rclcpp::NodeOptions());

  executor.add_node(node->get_node_base_interface());

  while (rclcpp::ok()) {
    common::status_monitor.touch(__func__);
    executor.spin_once();
  }

  rclcpp::shutdown();

  return 0;
}
