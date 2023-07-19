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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>

namespace traffic_simulator
{
class ConventionalTrafficLightManager : public TrafficLightManagerBase
{
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface_;

  rclcpp::PublisherBase::SharedPtr publisher_ = nullptr;

  using OldMessageType = autoware_auto_perception_msgs::msg::TrafficSignalArray;

  const std::string old_message_type_name_ =
    "autoware_auto_perception_msgs::msg::TrafficSignalArray";

  const std::string old_topic_name_;

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
  using NewMessageType = autoware_perception_msgs::msg::TrafficSignalArray;

  const std::string new_message_type_name_ = "autoware_perception_msgs::msg::TrafficSignalArray";

  const std::string new_topic_name_;
#endif

  bool use_new_interface_ = false;

public:
  template <typename Node>
  explicit ConventionalTrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap, const Node & node,
    const std::string & map_frame = "map")
  : TrafficLightManagerBase(node, hdmap, map_frame),
    node_topics_interface_(node->get_node_topics_interface()),
    node_parameters_interface_(node->get_node_parameters_interface()),
    old_topic_name_("/perception/traffic_light_recognition/traffic_signals")
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
    ,
    new_topic_name_("/perception/traffic_light_recognition/traffic_signals")
#endif
  {
  }

private:
  auto publishTrafficLightStateArray() const -> void override
  {
    if (use_new_interface_) {
#if __has_include(<autoware_perception_msgs/msg/traffic_signal.hpp>)
      NewMessageType new_message;
        static auto new_publisher = std::dynamic_pointer_cast<rclcpp::Publisher<NewMessageType>>(publisher_);
        new_message.stamp = clock_ptr_->now();
        for (const auto & [id, traffic_light] : getTrafficLights()) {
          new_message.signals.push_back(
            static_cast<typename NewMessageType::_signals_type::value_type>(traffic_light));
        }
        new_publisher->publish(new_message);
#endif
    } else {
        static auto old_publisher = std::dynamic_pointer_cast<rclcpp::Publisher<OldMessageType>>(publisher_);
      OldMessageType old_message;
        old_message.header.frame_id = "camera_link";  // DIRTY HACK!!!
        old_message.header.stamp = clock_ptr_->now();
        for (const auto & [id, traffic_light] : getTrafficLights()) {
          old_message.signals.push_back(
            static_cast<typename OldMessageType::_signals_type::value_type>(traffic_light));
        }
            old_publisher->publish(old_message);
    }
  }

  void initialize() override
  {
    if (auto old_topic_types = getTopicTypeList(old_topic_name_);
        std::find(old_topic_types.begin(), old_topic_types.end(), old_message_type_name_) !=
        old_topic_types.end()) {
      use_new_interface_ = false;
      publisher_ =
        std::static_pointer_cast<rclcpp::PublisherBase>(rclcpp::create_publisher<OldMessageType>(
          node_parameters_interface_, old_topic_name_, rclcpp::QoS(10).transient_local()));
    }
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
    else if (auto new_topic_types = getTopicTypeList(new_topic_name_);
             std::find(new_topic_types.begin(), new_topic_types.end(), new_message_type_name_) !=
             new_topic_types.end()) {
      use_new_interface_ = true;
      publisher_ =
        std::static_pointer_cast<rclcpp::PublisherBase>(rclcpp::create_publisher<NewMessageType>(
          node_parameters_interface_, new_topic_name_, rclcpp::QoS(10).transient_local()));
    }
#endif
    else {
      has_initialized_ = false;
    }
  }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_
