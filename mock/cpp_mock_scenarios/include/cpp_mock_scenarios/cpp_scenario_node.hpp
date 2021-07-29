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

#ifndef CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
#define CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

namespace cpp_mock_scenarios
{
class CppScenarioNode : public rclcpp::Node
{
public:
  explicit CppScenarioNode(
    const std::string & node_name, const std::string & map_path,
    const std::string & scenario_filename, const bool verbose, const rclcpp::NodeOptions & option);
  void start();
  void stop(bool success=true);
protected:
  traffic_simulator::API api_;
private:
  virtual void onUpdate() = 0;
  virtual void onInitialize() = 0;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

static auto configure(
  const std::string & map_path, const std::string & scenario_filename, const bool verbose)
  -> traffic_simulator::Configuration;
}  // namespace cpp_mock_scenarios

#endif  // CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
