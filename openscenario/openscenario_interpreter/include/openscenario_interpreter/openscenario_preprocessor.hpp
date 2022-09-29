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

#include <memory>
#include <openscenario_interpreter_msgs/srv/preprocessor_derive.hpp>
#include <openscenario_interpreter_msgs/srv/preprocessor_load.hpp>
#include <rclcpp/rclcpp.hpp>

namespace openscenario_interpreter
{
class Preprocessor : public rclcpp::Node
{
public:
  //  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Preprocessor(const rclcpp::NodeOptions & options);

private:
  void createDeriveServer();

  void destroyDeriveServer() { derive_server = nullptr; }

  rclcpp::Service<openscenario_interpreter_msgs::srv::PreprocessorLoad>::SharedPtr load_server;

  rclcpp::Service<openscenario_interpreter_msgs::srv::PreprocessorDerive>::SharedPtr derive_server;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_PREPROCESSOR_HPP_
