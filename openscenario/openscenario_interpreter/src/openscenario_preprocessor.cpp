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

//#define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#include <algorithm>
#include <openscenario_interpreter/openscenario_preprocessor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace openscenario_interpreter
{
Preprocessor::Preprocessor(const rclcpp::NodeOptions & options)
: rclcpp::Node("preprocessor", options)
{
  using openscenario_interpreter_msgs::srv::PreprocessorLoad;
  load_server = create_service<PreprocessorLoad>(
    "load",
    [this](
      const PreprocessorLoad::Request::SharedPtr request,
      PreprocessorLoad::Response::SharedPtr response) -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      try {
        auto s = ScenarioInfo(*request);
        preprocessScenario(s);
        response->has_succeeded = true;
        response->message = "success";
      } catch (std::exception & e) {
        response->has_succeeded = false;
        response->message = e.what();
        preprocessed_scenarios.clear();
      }
    });

  using openscenario_interpreter_msgs::srv::PreprocessorDerive;

  derive_server = create_service<PreprocessorDerive>(
    "derive",
    [this](
      [[maybe_unused]] const PreprocessorDerive::Request::SharedPtr request,
      PreprocessorDerive::Response::SharedPtr response) -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      if (preprocessed_scenarios.empty()) {
        response->path = "no output";
      } else {
        *response = preprocessed_scenarios.front().getDeriveResponse();
        response->expect = preprocessed_scenarios.size();
        preprocessed_scenarios.pop_front();
      }
    });

  using openscenario_interpreter_msgs::srv::PreprocessorCheckDerivationCompleted;

  check_server = create_service<PreprocessorCheckDerivationCompleted>(
    "check",
    [this](
      [[maybe_unused]] const PreprocessorCheckDerivationCompleted::Request::SharedPtr request,
      PreprocessorCheckDerivationCompleted::Response::SharedPtr response) -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      response->derivation_completed = (preprocessed_scenarios.empty());
    });
}
bool Preprocessor::validateXOSC(const std::string file_name)
{
  return concealer::dollar("ros2 run openscenario_utility validate-xosc " + file_name)
           .find("All xosc files given are standard compliant.") != std::string::npos;
}
void Preprocessor::preprocessScenario(ScenarioInfo & scenario)
{
  // this function doesn't support ParameterValueDistribution now
  if (validateXOSC(scenario.path)) {
    //      auto script = OpenScenario(scenario.path);
    //  if (hasElement("ParameterValueDistribution", scenario.path)) {
    //      assert( validateXOSC( linked scenario.path );
    //  auto derive_server = createDeriveServer();
    //      parameters = evaluate( parameter_value_distribution( given scenario ) )
    //      for( auto derived_scenario : embedParameter( linked scenario, parameters)
    //        preprocessed_scenarios.emplace_back({derived_scenario, derive_server});
    preprocessed_scenarios.emplace_back(scenario);  // temporary code
  } else {
    throw common::Error("the scenario file is not valid. Please check your scenario");
  }
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Preprocessor)
