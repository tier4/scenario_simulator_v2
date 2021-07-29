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

namespace cpp_mock_scenarios
{
CppScenarioNode::CppScenarioNode(
  const std::string & node_name, const std::string & map_path,
  const std::string & scenario_filename, const bool verbose, const rclcpp::NodeOptions & option)
: Node(node_name, option), api_(this, configure(map_path, scenario_filename, verbose))
{
}

void CppScenarioNode::start()
{
  onInitialize();
  using namespace std::chrono_literals;
  update_timer_ = this->create_wall_timer(50ms, std::bind(&CppScenarioNode::onUpdate, this));
}

static auto configure(
  const std::string & map_path, const std::string & scenario_filename, const bool verbose)
  -> traffic_simulator::Configuration
{
  auto configuration = traffic_simulator::Configuration(map_path);
  {
    configuration.lanelet2_map_file = "lanelet2_map_with_private_road_and_walkway_ele_fix.osm";
    configuration.scenario_path = scenario_filename;
    configuration.verbose = verbose;
    configuration.initialize_duration = 0;
  }
  return configuration;
}
}  // namespace cpp_mock_scenarios
