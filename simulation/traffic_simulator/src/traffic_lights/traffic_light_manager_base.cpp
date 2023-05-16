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

#include <iterator>
#include <memory>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace traffic_simulator
{
auto TrafficLightManagerBase::deleteAllMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray message;
  {
    visualization_msgs::msg::Marker marker;
    marker.action = marker.DELETEALL;
    message.markers.push_back(marker);
  }

  marker_pub_->publish(message);
}

auto TrafficLightManagerBase::drawMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto now = clock_ptr_->now();

  for (const auto & [id, traffic_light] : getTrafficLights()) {
    traffic_light.draw(marker_array.markers, now, map_frame_);
  }

  marker_pub_->publish(marker_array);
}

auto TrafficLightManagerBase::hasAnyLightChanged() -> bool
{
  return true;
  // return std::any_of(
  //   std::begin(getTrafficLights()), std::end(getTrafficLights()), [](auto && id_and_traffic_light) {
  //     return id_and_traffic_light.second.colorChanged() or
  //            id_and_traffic_light.second.arrowChanged();
  //   });
}

auto TrafficLightManagerBase::update(const double) -> void
{
  publishTrafficLightStateArray();

  if (hasAnyLightChanged()) {
    deleteAllMarkers();
  }

  drawMarkers();
}

auto TrafficLightManagerBase::createTimer(double publish_rate) -> void
{
  if (!timer_) {
    publish_rate_ = publish_rate;
    using namespace std::chrono_literals;
    timer_ = rclcpp::create_timer(
      node_base_interface_, node_timers_interface_, clock_ptr_, 1s / publish_rate_,
      [this]() -> void { update(1.0 / publish_rate_); });
  }
}

auto TrafficLightManagerBase::applyPublishRate(double publish_rate) -> void
{
  if (publish_rate_ != publish_rate) {
    publish_rate_ = publish_rate;
    if (timer_ && not timer_->is_canceled()) {
      timer_->cancel();
    }

    using namespace std::chrono_literals;
    timer_ = rclcpp::create_timer(
      node_base_interface_, node_timers_interface_, clock_ptr_, 1s / publish_rate_,
      [this]() -> void { update(1.0 / publish_rate_); });
  }
}
}  // namespace traffic_simulator
