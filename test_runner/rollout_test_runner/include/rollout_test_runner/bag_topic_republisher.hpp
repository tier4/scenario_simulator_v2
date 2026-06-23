// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#ifndef ROLLOUT_TEST_RUNNER__BAG_TOPIC_REPUBLISHER_HPP_
#define ROLLOUT_TEST_RUNNER__BAG_TOPIC_REPUBLISHER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace rollout_test_runner
{
/// Republishes selected rosbag topics with their original timestamps, paced by simulation time.
class BagTopicRepublisher
{
public:
  BagTopicRepublisher(
    rclcpp::Node & node, const std::string & bag_path, const std::vector<std::string> & topics,
    const std::vector<std::string> & transient_local_topics);

  /// Publishes all messages recorded at or before the given bag time. Returns the publish count.
  auto publishUntil(const rclcpp::Time & bag_time) -> std::size_t;

private:
  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
  std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
  rosbag2_storage::SerializedBagMessageSharedPtr pending_;
};
}  // namespace rollout_test_runner

#endif  // ROLLOUT_TEST_RUNNER__BAG_TOPIC_REPUBLISHER_HPP_
