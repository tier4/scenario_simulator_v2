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

#include <cstdlib>
#include <memory>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>
#include <openscenario_preprocessor_msgs/srv/check_derivative_remained.hpp>
#include <openscenario_preprocessor_msgs/srv/derive.hpp>
#include <openscenario_preprocessor_msgs/srv/load.hpp>
#include <rclcpp/rclcpp.hpp>

class PreprocessorNode : public rclcpp::Node, public openscenario_preprocessor::Preprocessor
{
public:
  explicit PreprocessorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("openscenario_preprocessor", options),
    openscenario_preprocessor::Preprocessor([this] {
      declare_parameter<std::string>("output_directory", "/tmp/openscenario_preprocessor");
      return get_parameter("output_directory").as_string();
    }()),
    load_server(create_service<openscenario_preprocessor_msgs::srv::Load>(
      "~/load",
      [this](
        const openscenario_preprocessor_msgs::srv::Load::Request::SharedPtr request,
        openscenario_preprocessor_msgs::srv::Load::Response::SharedPtr response) -> void {
        auto lock = std::lock_guard(preprocessed_scenarios_mutex);
        try {
          preprocessScenario(
            openscenario_preprocessor::Scenario(request->path, request->frame_rate));
          response->has_succeeded = true;
          response->message = "success";
        } catch (std::exception & e) {
          response->has_succeeded = false;
          response->message = e.what();
          std::queue<openscenario_preprocessor::Scenario>().swap(preprocessed_scenarios);
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
          response->path = preprocessed_scenarios.front().path.string();
          response->frame_rate = preprocessed_scenarios.front().frame_rate;
          preprocessed_scenarios.pop();
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

private:
  rclcpp::Service<openscenario_preprocessor_msgs::srv::Load>::SharedPtr load_server;

  rclcpp::Service<openscenario_preprocessor_msgs::srv::Derive>::SharedPtr derive_server;

  rclcpp::Service<openscenario_preprocessor_msgs::srv::CheckDerivativeRemained>::SharedPtr
    check_server;
};

int main(const int argc, char const * const * const argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor{};

  rclcpp::NodeOptions options{};

  auto node = std::make_shared<PreprocessorNode>(options);

  executor.add_node((*node).get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
