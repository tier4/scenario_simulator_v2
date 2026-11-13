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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP

#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace traffic_simulator
{
class ConfigurableRateUpdater
{
public:
  template <typename NodePointer>
  ConfigurableRateUpdater(const NodePointer & node, std::function<void()> thunk)
  : node_base_interface_(node->get_node_base_interface()),
    node_timers_interface_(node->get_node_timers_interface()),
    clock_ptr_(node->get_clock()),
    thunk_(std::move(thunk))
  {
  }

  auto startTimer(const double update_rate) -> void;

  auto resetTimer(const double update_rate) -> void;

  auto getUpdateRate() const -> double { return update_rate_; }

private:
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_interface_;
  const rclcpp::Clock::SharedPtr clock_ptr_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  const std::function<void()> thunk_;
  double update_rate_ = 0.0;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONFIGURABLE_RATE_UPDATER_HPP
