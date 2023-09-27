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

#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>

namespace traffic_simulator
{
auto ConfigurableRateUpdater::createTimer(double update_rate) -> void
{
  if (!timer_) {
    update_rate_ = update_rate;
    using namespace std::chrono_literals;
    timer_ = rclcpp::create_timer(
      node_base_interface_, node_timers_interface_, clock_ptr_, 1s / update_rate_,
      [this]() -> void { thunk_(); });
  }
}

auto ConfigurableRateUpdater::resetUpdateRate(double update_rate) -> void
{
  if (update_rate_ != update_rate) {
    update_rate_ = update_rate;
    if (timer_ && not timer_->is_canceled()) {
      timer_->cancel();
    }

    using namespace std::chrono_literals;
    timer_ = rclcpp::create_timer(
      node_base_interface_, node_timers_interface_, clock_ptr_, 1s / update_rate_,
      [this]() -> void { thunk_(); });
  }
}
}  // namespace traffic_simulator
