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

#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <iostream>

namespace cpp_mock_scenarios
{
CppScenarioNode::CppScenarioNode(
  const std::string & node_name, const std::string & map_path,
  const std::string & lanelet2_map_file, const std::string & scenario_filename, const bool verbose,
  const rclcpp::NodeOptions & option)
: Node(node_name, option),
  api_(this, configure(map_path, lanelet2_map_file, scenario_filename, verbose)),
  exception_expect_(false)
{
}

void CppScenarioNode::update()
{
  onUpdate();
  try {
    api_.updateFrame();
  } catch (const common::Error & e) {
    if (exception_expect_) {
      stop(Result::SUCCESS);
    } else {
      stop(Result::FAILURE);
    }
  }
}

void CppScenarioNode::start()
{
  api_.initialize(1.0, 0.05);
  onInitialize();
  using namespace std::chrono_literals;
  update_timer_ = this->create_wall_timer(50ms, std::bind(&CppScenarioNode::update, this));
}

void CppScenarioNode::stop(Result result)
{
  switch (result) {
    case Result::SUCCESS: {
      std::cout << "cpp_scenario:success" << std::endl;
      break;
    }
    case Result::FAILURE: {
      std::cerr << "cpp_scenario:failure" << std::endl;
      break;
    }
  }
  update_timer_->cancel();
  rclcpp::shutdown();
  std::exit(0);
}

void CppScenarioNode::checkConfiguration(const traffic_simulator::Configuration & configuration)
{
  try {
    configuration.getLanelet2MapFile();
    configuration.getPointCloudMapFile();
  } catch (const common::SimulationError &) {
    stop(Result::FAILURE);
  }
}
}  // namespace cpp_mock_scenarios
