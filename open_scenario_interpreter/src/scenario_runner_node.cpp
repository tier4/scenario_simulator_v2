// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#define SCENARIO_RUNNER_ALLOW_ATTRIBUTES_TO_BE_BLANK
// #define SCENARIO_RUNNER_NO_EXTENSION

#include <scenario_runner/scenario_runner.hpp>

#include <cstdlib>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor {};

  rclcpp::NodeOptions options {};

  auto node {
    std::make_shared<scenario_runner::ScenarioRunner>(options)
  };

  executor.add_node((*node).get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
