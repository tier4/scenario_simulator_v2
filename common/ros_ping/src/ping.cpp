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

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

enum class ReturnCode {
  success = 0,
  connect_server_timeout = 1,
  request_timeout = 2,
  request_interrupted = 3,
  unknown = 4,
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ping");

  node->declare_parameter<std::string>("service_name", "/ping");
  std::string service_name;
  node->get_parameter<std::string>("service_name", service_name);

  node->declare_parameter<int>("connection_timeout_ms", 1000);
  int connection_timeout_ms;
  node->get_parameter<int>("connection_timeout_ms", connection_timeout_ms);

  node->declare_parameter<int>("request_timeout_ms", 1000);
  int request_timeout_ms;
  node->get_parameter<int>("request_timeout_ms", request_timeout_ms);

  auto ping_client = node->create_client<std_srvs::srv::Empty>(service_name);

  using namespace std::chrono_literals;
  if (not ping_client->wait_for_service(1ms * connection_timeout_ms)) {
    return static_cast<int>(ReturnCode::connect_server_timeout);
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  auto response_future = ping_client->async_send_request(request);

  auto return_code = rclcpp::spin_until_future_complete(node, response_future, 1ms * request_timeout_ms);

  rclcpp::shutdown();

  switch (return_code) {
    case rclcpp::FutureReturnCode::SUCCESS:
      return static_cast<int>(ReturnCode::success);
    case rclcpp::FutureReturnCode::INTERRUPTED:
      return static_cast<int>(ReturnCode::request_interrupted);
    case rclcpp::FutureReturnCode::TIMEOUT:
      return static_cast<int>(ReturnCode::request_timeout);
    default:
      return static_cast<int>(ReturnCode::unknown);
  }
}
