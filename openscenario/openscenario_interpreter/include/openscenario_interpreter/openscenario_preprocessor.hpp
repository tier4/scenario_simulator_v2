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
#include <openscenario_interpreter_msgs/srv/preprocess.hpp>
#include <rclcpp/rclcpp.hpp>

namespace openscenario_interpreter
{
class Preprocessor : rclcpp::Node
{
public:
  //  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Preprocessor(const rclcpp::NodeOptions & options) : rclcpp::Node("preprocessor", options)
  {
    using openscenario_interpreter_msgs::srv::Preprocess;
    auto handle_preprocess = [this](
                               const Preprocess::Request::SharedPtr request,
                               Preprocess::Response::SharedPtr response) -> void {
      // TODO: implement
    };
    server = create_service<Preprocess>("preprocess", handle_preprocess);
  }

  ~Preprocessor();

private:
  rclcpp::Service<openscenario_interpreter_msgs::srv::Preprocess>::SharedPtr server;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_PREPROCESSOR_HPP_
