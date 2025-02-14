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

#ifndef OPENSCENARIO_INTERPRETER__VISUALIZATION_BUFFER_HPP_
#define OPENSCENARIO_INTERPRETER__VISUALIZATION_BUFFER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace openscenario_interpreter
{
class VisualizationBuffer
{
  static inline std::unique_ptr<VisualizationBuffer> buffer = nullptr;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
  std::vector<visualization_msgs::msg::Marker> markers;

public:
  template <typename Node>
  VisualizationBuffer(Node && node, const std::string topic = "/simulation/interpreter/markers")
  {
    publisher = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, topic, rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  static auto active() { return static_cast<bool>(buffer); }

  template <typename Node>
  static auto activate(const Node & node) -> void
  {
    if (not active()) {
      buffer = std::make_unique<VisualizationBuffer>(node);
    } else {
      throw Error("The visualization buffer has already been instantiated.");
    }
  }

  static auto deactivate() -> void
  {
    if (active()) {
      buffer.reset();
    }
  }

  static auto flush() -> void
  {
    if (active()) {
      visualization_msgs::msg::MarkerArray message;
      message.markers = buffer->markers;
      buffer->publisher->publish(message);
      buffer->markers.clear();
    }
  }

  class Target
  {
  public:
    void add(const visualization_msgs::msg::Marker & marker) const
    {
      if (active()) {
        buffer->markers.push_back(marker);
      }
    }

    template <typename F>
    void call_visualize(F && function) const
    {
      if (active()) {
        function();
      }
    }
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__VISUALIZATION_BUFFER_HPP_
