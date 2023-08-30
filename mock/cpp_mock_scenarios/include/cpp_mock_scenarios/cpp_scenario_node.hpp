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

#ifndef CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
#define CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <simple_junit/junit5.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace cpp_mock_scenarios
{
enum class Result { SUCCESS = 0, FAILURE = 1 };

class CppScenarioNode : public rclcpp::Node
{
public:
  explicit CppScenarioNode(
    const std::string & node_name, const std::string & map_path,
    const std::string & lanelet2_map_file, const std::string & scenario_filename,
    const bool verbose, const rclcpp::NodeOptions & option);
  void start();
  void stop(Result result, const std::string & description = "");
  void expectThrow() { exception_expect_ = true; }
  void expectNoThrow() { exception_expect_ = false; }
  template <typename T>
  auto equals(const T v1, const T v2, const T tolerance = std::numeric_limits<T>::epsilon()) const
    -> bool
  {
    if (std::abs(v2 - v1) <= tolerance) {
      return true;
    }
    return false;
  }

protected:
  traffic_simulator::API api_;
  common::junit::JUnit5 junit_;

private:
  std::string scenario_filename_;
  bool exception_expect_;
  std::string junit_path_;
  void update();
  virtual void onUpdate() = 0;
  virtual void onInitialize() = 0;
  rclcpp::TimerBase::SharedPtr update_timer_;
  double timeout_;
  auto configure(
    const std::string & map_path, const std::string & lanelet2_map_file,
    const std::string & scenario_filename, const bool verbose) -> traffic_simulator::Configuration
  {
    auto configuration = traffic_simulator::Configuration(map_path);
    {
      configuration.lanelet2_map_file = lanelet2_map_file;
      // configuration.lanelet2_map_file = "lanelet2_map_with_private_road_and_walkway_ele_fix.osm";
      configuration.scenario_path = scenario_filename;
      configuration.verbose = verbose;
    }
    checkConfiguration(configuration);
    return configuration;
  }

  void checkConfiguration(const traffic_simulator::Configuration & configuration);
};

}  // namespace cpp_mock_scenarios

#endif  // CPP_MOCK_SCENARIOS__CPP_SCENARIO_NODE_HPP_
