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

enum class ReturnCode {
  success,
  timeout,
  no_publishers,
  interrupted,
  unknown,
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("topic_status_checker");

  node->declare_parameter<std::string>("topic_name", "");
  std::string topic_name;
  node->get_parameter<std::string>("topic_name", topic_name);

  node->declare_parameter<int>("timeout_ms", 2000);
  int timeout_ms;
  node->get_parameter<int>("timeout_ms", timeout_ms);

  int topic_discovery_time_ms;
  node->declare_parameter<int>("topic_discovery_time_ms", 1000);
  node->get_parameter<int>("topic_discovery_time_ms", topic_discovery_time_ms);

  bool verbose;
  node->declare_parameter<bool>("verbose", true);
  node->get_parameter<bool>("verbose", verbose);

  // wait for topic discovery
  // ref : https://github.com/ros2/ros2/issues/1057
  using namespace std::chrono_literals;
  rclcpp::sleep_for(1ms * topic_discovery_time_ms);

  auto topic_names_and_types = node->get_topic_names_and_types();
  if (auto topic_info = topic_names_and_types.find(topic_name);
      topic_info != topic_names_and_types.end()) {
    if (verbose) {
      RCLCPP_INFO_STREAM(node->get_logger(), "The topic `" << topic_name << "` is found!");
    }
    std::promise<void> topic_notifier;
    auto topic_monitor = topic_notifier.get_future();

    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
    options.callback_group = callback_group;

    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions;
    for (auto topic_type_string : topic_info->second) {
      subscriptions.push_back(node->create_generic_subscription(
        topic_name, topic_type_string, rclcpp::QoS(10),
        [&topic_notifier](std::shared_ptr<rclcpp::SerializedMessage>) {
          topic_notifier.set_value();
        },
        options));
    }

    switch (rclcpp::spin_until_future_complete(node, topic_monitor, 1ms * timeout_ms)) {
      case rclcpp::FutureReturnCode::SUCCESS:
        if (verbose) {
          RCLCPP_INFO_STREAM(
            node->get_logger(), "Receive a message from `" << topic_name << "` topic!");
        }
        return static_cast<int>(ReturnCode::success);
      case rclcpp::FutureReturnCode::TIMEOUT:
        if (verbose) {
          RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Timeout for receiving a message from `" << topic_name << "` topic...");
          RCLCPP_INFO_STREAM(node->get_logger(), "Timeout duration : " << timeout_ms);
        }
        return static_cast<int>(ReturnCode::timeout);
      case rclcpp::FutureReturnCode::INTERRUPTED:
        if (verbose) {
          RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Interrupted to receive a message from `" << topic_name << "` topic...");
        }
        return static_cast<int>(ReturnCode::interrupted);
      default:
        if (verbose) {
          RCLCPP_INFO_STREAM(
            node->get_logger(), "Unknown error is occurred during receiving a message from `"
                                  << topic_name << "` topic...");
        }
        return static_cast<int>(ReturnCode::unknown);
    }
  } else {
    if (verbose) {
      RCLCPP_INFO_STREAM(node->get_logger(), "The topic `" << topic_name << "` is not found!");
    }
    return static_cast<int>(ReturnCode::no_publishers);
  }
}
