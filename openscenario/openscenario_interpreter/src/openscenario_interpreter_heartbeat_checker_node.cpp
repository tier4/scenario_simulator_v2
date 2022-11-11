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

#include <glog/logging.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
//#include <openscenario_interpreter/openscenario_interpreter.hpp>

enum class Return {
  SUCCESS = 0,
  CONNECT_SERVER_TIMEOUT = 1,
  REQUEST_TIMEOUT = 2,
  REQUEST_INTERRUPTED = 3,
  UNKNOWN = 4,
};

int main(const int argc, char const * const * const argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("heartbeat_checker");

  auto heartbeat_client = node->create_client<std_srvs::srv::Empty>("/simulation/heartbeat");

  using namespace std::chrono_literals;
  if (heartbeat_client->wait_for_service(1s)) {
    return static_cast<int>(Return::CONNECT_SERVER_TIMEOUT);
  }
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  auto response_future = heartbeat_client->async_send_request(request);

  auto return_code = rclcpp::spin_until_future_complete(node, response_future, 1s);
  rclcpp::shutdown();

  switch (return_code) {
    case rclcpp::FutureReturnCode::SUCCESS:
      return static_cast<int>(Return::SUCCESS);
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      return static_cast<int>(Return::REQUEST_INTERRUPTED);
      break;
    case rclcpp::FutureReturnCode::TIMEOUT:
      return static_cast<int>(Return::REQUEST_TIMEOUT);
      break;
    default:
      return static_cast<int>(Return::UNKNOWN);
  }
}
