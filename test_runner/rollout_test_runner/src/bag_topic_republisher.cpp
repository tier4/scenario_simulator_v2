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

#include <algorithm>
#include <iomanip>
#include <rollout_test_runner/bag_topic_republisher.hpp>

namespace rollout_test_runner
{
namespace
{
/// Topics supplied by the simulator itself must never be replayed from the bag.
auto isForbidden(const std::string & topic) -> bool
{
  const auto starts_with = [&](const std::string & prefix) {
    return topic.rfind(prefix, 0) == 0;
  };
  return topic == "/tf" or topic == "/tf_static" or topic == "/clock" or
         starts_with("/localization/") or starts_with("/vehicle/status/");
}
}  // namespace

BagTopicRepublisher::BagTopicRepublisher(
  rclcpp::Node & node, const std::string & bag_path, const std::vector<std::string> & topics,
  const std::vector<std::string> & transient_local_topics)
{
  std::vector<std::string> accepted_topics;
  for (const auto & topic : topics) {
    if (isForbidden(topic)) {
      RCLCPP_WARN_STREAM(
        node.get_logger(),
        "Topic " << std::quoted(topic)
                 << " is supplied by the simulator and removed from passthrough topics.");
    } else {
      accepted_topics.push_back(topic);
    }
  }

  reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  reader_->open(storage_options, rosbag2_cpp::ConverterOptions());

  for (const auto & topic_metadata : reader_->get_all_topics_and_types()) {
    if (
      std::find(accepted_topics.begin(), accepted_topics.end(), topic_metadata.name) !=
      accepted_topics.end()) {
      const auto transient_local =
        std::find(
          transient_local_topics.begin(), transient_local_topics.end(), topic_metadata.name) !=
        transient_local_topics.end();
      const auto qos = transient_local ? rclcpp::QoS(1).reliable().transient_local()
                                       : rclcpp::QoS(10).reliable();
      publishers_.emplace(
        topic_metadata.name,
        node.create_generic_publisher(topic_metadata.name, topic_metadata.type, qos));
    }
  }

  for (const auto & topic : accepted_topics) {
    if (publishers_.find(topic) == publishers_.end()) {
      RCLCPP_WARN_STREAM(
        node.get_logger(),
        "Passthrough topic " << std::quoted(topic) << " was not found in rosbag "
                             << std::quoted(bag_path) << ".");
    }
  }

  rosbag2_storage::StorageFilter filter;
  for (const auto & [topic, publisher] : publishers_) {
    filter.topics.push_back(topic);
  }
  reader_->set_filter(filter);
}

auto BagTopicRepublisher::publishUntil(const rclcpp::Time & bag_time) -> std::size_t
{
  const auto bag_time_ns = bag_time.nanoseconds();
  std::size_t count = 0;
  while (true) {
    if (not pending_) {
      if (not reader_->has_next()) {
        return count;
      }
      pending_ = reader_->read_next();
    }
    if (pending_->time_stamp > bag_time_ns) {
      return count;
    }
    if (const auto iterator = publishers_.find(pending_->topic_name);
        iterator != publishers_.end()) {
      iterator->second->publish(rclcpp::SerializedMessage(*pending_->serialized_data));
      ++count;
    }
    pending_.reset();
  }
}
}  // namespace rollout_test_runner
