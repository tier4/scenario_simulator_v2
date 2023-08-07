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

#ifndef OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
#define OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_

#include <concealer/execute.hpp>
#include <deque>
#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_preprocessor_msgs/srv/check_derivative_remained.hpp>
#include <openscenario_preprocessor_msgs/srv/derive.hpp>
#include <openscenario_preprocessor_msgs/srv/load.hpp>
#include <rclcpp/rclcpp.hpp>

namespace openscenario_preprocessor
{
struct ScenarioSet
{
  ScenarioSet() = default;

  explicit ScenarioSet(openscenario_preprocessor_msgs::srv::Load::Request & load_request)
  {
    path = load_request.path;
    frame_rate = load_request.frame_rate;
  }

  auto getDeriveResponse() -> openscenario_preprocessor_msgs::srv::Derive::Response
  {
    openscenario_preprocessor_msgs::srv::Derive::Response response;
    response.path = path;
    response.frame_rate = frame_rate;
    return response;
  }

  std::string path;

  float frame_rate;
};

class Preprocessor : public rclcpp::Node
{
public:
  explicit Preprocessor(const rclcpp::NodeOptions &);

private:
  void preprocessScenario(ScenarioSet &);

  [[nodiscard]] bool validateXOSC(const boost::filesystem::path &, bool);

  rclcpp::Service<openscenario_preprocessor_msgs::srv::Load>::SharedPtr load_server;

  rclcpp::Service<openscenario_preprocessor_msgs::srv::Derive>::SharedPtr derive_server;

  rclcpp::Service<openscenario_preprocessor_msgs::srv::CheckDerivativeRemained>::SharedPtr
    check_server;

  std::deque<ScenarioSet> preprocessed_scenarios;

  std::mutex preprocessed_scenarios_mutex;
};
}  // namespace openscenario_preprocessor

#endif  // OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
