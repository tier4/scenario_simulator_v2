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

#ifndef CONCEALER__PUBLISHER_HPP_
#define CONCEALER__PUBLISHER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
template <typename Message>
class Publisher
{
  typename rclcpp::Publisher<Message>::SharedPtr publisher;

public:
  template <typename Node>
  explicit Publisher(const std::string & topic, Node & node)
  : publisher(node.template create_publisher<Message>(topic, rclcpp::QoS(1).reliable()))
  {
  }

  template <typename... Ts>
  auto operator()(Ts &&... xs) const -> decltype(auto)
  {
    return publisher->publish(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace concealer

#endif  // CONCEALER__PUBLISHER_HPP_
