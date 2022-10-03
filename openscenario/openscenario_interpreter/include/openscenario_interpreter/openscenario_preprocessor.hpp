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

#ifndef OPENSCENARIO_INTERPRETER__OPENSCENARIO_PREPROCESSOR_HPP_
#define OPENSCENARIO_INTERPRETER__OPENSCENARIO_PREPROCESSOR_HPP_

#include <concealer/execute.hpp>
#include <deque>
#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter_msgs/srv/preprocessor_check_derivative_remained.hpp>
#include <openscenario_interpreter_msgs/srv/preprocessor_derive.hpp>
#include <openscenario_interpreter_msgs/srv/preprocessor_load.hpp>
#include <rclcpp/rclcpp.hpp>

namespace openscenario_interpreter
{

struct ScenarioInfo
{
  ScenarioInfo() {}
  ScenarioInfo(openscenario_interpreter_msgs::srv::PreprocessorLoad::Request & load_request)
  {
    path = load_request.path;
    expect = load_request.expect;
    frame_rate = load_request.frame_rate;
  }
  auto getDeriveResponse() -> openscenario_interpreter_msgs::srv::PreprocessorDerive::Response
  {
    openscenario_interpreter_msgs::srv::PreprocessorDerive::Response response;
    response.path = path;
    response.expect = expect;
    response.frame_rate = frame_rate;
    return response;
  }
  std::string path;
  int expect;
  float frame_rate;
};

class Preprocessor : public rclcpp::Node
{
public:
  //  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Preprocessor(const rclcpp::NodeOptions & options);

private:
  void preprocessScenario(ScenarioInfo & scenario);

  [[nodiscard]] bool validateXOSC(const std::string file_name);

  rclcpp::Service<openscenario_interpreter_msgs::srv::PreprocessorLoad>::SharedPtr load_server;

  rclcpp::Service<openscenario_interpreter_msgs::srv::PreprocessorDerive>::SharedPtr derive_server;

  rclcpp::Service<openscenario_interpreter_msgs::srv::PreprocessorCheckDerivativeRemained>::
    SharedPtr check_server;

  std::deque<ScenarioInfo> preprocessed_scenarios;

  std::mutex preprocessed_scenarios_mutex;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_PREPROCESSOR_HPP_
