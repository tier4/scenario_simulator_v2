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

#ifndef ROS_PING__SERVER_HPP_
#define ROS_PING__SERVER_HPP_

#include <std_srvs/srv/empty.hpp>

namespace ros_ping
{
class PingServer
{
private:
  std::shared_ptr<std::promise<void>> update_notifier = nullptr;

  const rclcpp::CallbackGroup::SharedPtr service_callback_group;

  const rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;

public:
  template <typename NodeT>
  explicit PingServer(NodeT & node)
  : service_callback_group(
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    service(node.template create_service<std_srvs::srv::Empty>(
      "ping",
      [this](
        const std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr) -> void {
        if (not update_notifier) {
          update_notifier = std::make_shared<std::promise<void>>();
          auto future = update_notifier->get_future();
          future.get();
          update_notifier.reset();
        } else {
          throw std::runtime_error(
            "Multiple ping requests are accumulated. The loop that ping service is watching "
            "may not be responding");
        }
      },
      rmw_qos_profile_services_default, service_callback_group))
  {
  }

  void notifyAlive() const
  {
    if (update_notifier) {
      update_notifier->set_value();
    }
  }
};
}  // namespace ros_ping

#endif  // ROS_PING__SERVER_HPP_
