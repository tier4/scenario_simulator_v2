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

#include <algorithm>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace openscenario_preprocessor
{
Preprocessor::Preprocessor(const rclcpp::NodeOptions & options)
: rclcpp::Node("openscenario_preprocessor", options),
  load_server(create_service<openscenario_preprocessor_msgs::srv::Load>(
    "~/load",
    [this](
      const openscenario_preprocessor_msgs::srv::Load::Request::SharedPtr request,
      openscenario_preprocessor_msgs::srv::Load::Response::SharedPtr response) -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      try {
        auto s = ScenarioSet(*request);
        preprocessScenario(s);
        response->has_succeeded = true;
        response->message = "success";
      } catch (std::exception & e) {
        response->has_succeeded = false;
        response->message = e.what();
        preprocessed_scenarios.clear();
      }
    })),
  derive_server(create_service<openscenario_preprocessor_msgs::srv::Derive>(
    "~/derive",
    [this](
      const openscenario_preprocessor_msgs::srv::Derive::Request::SharedPtr,
      openscenario_preprocessor_msgs::srv::Derive::Response::SharedPtr response) -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      if (preprocessed_scenarios.empty()) {
        response->path = "no output";
      } else {
        *response = preprocessed_scenarios.front().getDeriveResponse();
        preprocessed_scenarios.pop_front();
      }
    })),
  check_server(create_service<openscenario_preprocessor_msgs::srv::CheckDerivativeRemained>(
    "~/check",
    [this](
      const openscenario_preprocessor_msgs::srv::CheckDerivativeRemained::Request::SharedPtr,
      openscenario_preprocessor_msgs::srv::CheckDerivativeRemained::Response::SharedPtr response)
      -> void {
      auto lock = std::lock_guard(preprocessed_scenarios_mutex);
      response->derivative_remained = not preprocessed_scenarios.empty();
    }))
{
}

bool Preprocessor::validateXOSC(const boost::filesystem::path & file_name, bool verbose = false)
{
  auto result =
    concealer::dollar("ros2 run openscenario_utility validation.py " + file_name.string());
  if (verbose) {
    std::cout << "validate : " << result << std::endl;
  }
  return result.find("All xosc files given are standard compliant.") != std::string::npos;
}

void Preprocessor::preprocessScenario(ScenarioSet & scenario)
{
  using openscenario_interpreter::OpenScenario;
  using openscenario_interpreter::ParameterValueDistribution;

  if (validateXOSC(scenario.path)) {
    if (auto script = std::make_shared<OpenScenario>(scenario.path);
        script->category.is<ParameterValueDistribution>()) {
      std::cout << "ParameterValueDistribution!!" << std::endl;
      auto base_scenario_path =
        script->category.as<ParameterValueDistribution>().scenario_file.filepath;
      std::cout << "base_scenario_path : " << base_scenario_path << std::endl;
      if (boost::filesystem::exists(base_scenario_path)) {
        if (validateXOSC(base_scenario_path, true)) {
          // TODO : implement in feature/parameter_value_distribution branch
          //
          //  parameters = evaluate( parameter_value_distribution( given scenario ) )
          //  for( auto derived_scenario : embedParameter( linked scenario, parameters)
          //    preprocessed_scenarios.emplace_back({derived_scenario, derive_server});
        } else {
          throw common::Error("base scenario is not valid : " + base_scenario_path.string());
        }
      } else {
        throw common::Error("base scenario does not exist : " + base_scenario_path.string());
      }
      std::cout << "base scenario is valid!" << std::endl;

    } else {
      // normal scenario
      preprocessed_scenarios.emplace_back(scenario);
    }
  } else {
    throw common::Error("the scenario file is not valid. Please check your scenario");
  }
}
}  // namespace openscenario_preprocessor

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_preprocessor::Preprocessor)
