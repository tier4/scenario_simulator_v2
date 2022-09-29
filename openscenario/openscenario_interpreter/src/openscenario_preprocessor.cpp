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
    // TODO: implement
  };
}
void Preprocessor::createDeriveServer()
{
  using openscenario_interpreter_msgs::srv::PreprocessorDerive;
  auto handle_derive = [this](
    const PreprocessorDerive::Request::SharedPtr request,
    PreprocessorDerive::Response::SharedPtr response) -> void {
    // TODO: implement
  };
  derive_server = create_service<PreprocessorDerive>("derive", handle_derive);
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Preprocessor)
