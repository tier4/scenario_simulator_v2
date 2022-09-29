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
  auto handle_load = [this](
                       const PreprocessorLoad::Request::SharedPtr request,
                       PreprocessorLoad::Response::SharedPtr response) -> void {
    auto lock = std::lock_guard(preprocessed_scenarios_mutex);
    try {
      preprocessScenario(ScenarioInfo(*request));

    } catch (...) {
      response->has_succeeded = false;
      response->message = "Something went wrong";
    }
  };
  load_server = create_service<PreprocessorLoad>("load", handle_load);

  using openscenario_interpreter_msgs::srv::PreprocessorDerive;
  auto handle_derive = [this](
                         const PreprocessorDerive::Request::SharedPtr request,
                         PreprocessorDerive::Response::SharedPtr response) -> void {
    auto lock = std::lock_guard(preprocessed_scenarios_mutex);
    if (preprocessed_scenarios.empty()) {
      response->path = "";
    } else {
      *response = preprocessed_scenarios.front().getDeriveResponse();
      preprocessed_scenarios.pop();
    }
  };

  derive_server = create_service<PreprocessorDerive>("derive", handle_derive);

  using openscenario_interpreter_msgs::srv::PreprocessorCheckDerivationCompleted;
  auto handle_check =
    [this](
      const PreprocessorCheckDerivationCompleted::Request::SharedPtr request,
      PreprocessorCheckDerivationCompleted::Response::SharedPtr response) -> void {
    auto lock = std::lock_guard(preprocessed_scenarios_mutex);
    response->derivation_completed = (preprocessed_scenarios.empty());
  };

  check_server = create_service<PreprocessorCheckDerivationCompleted>("check", handle_check);
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Preprocessor)
