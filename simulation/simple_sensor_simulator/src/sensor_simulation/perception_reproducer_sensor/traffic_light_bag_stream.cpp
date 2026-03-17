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

#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/traffic_light_bag_stream.hpp>

#ifdef PERCEPTION_REPRODUCER_HAS_TRAFFIC_LIGHT_GROUP_ARRAY

namespace simple_sensor_simulator
{
inline namespace experimental
{

TrafficLightBagStream::TrafficLightBagStream(
  const std::string & topic_name, rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr publisher)
: BagStreamBase<TrafficLightGroupArray>(topic_name), publisher_(publisher)
{
}

auto TrafficLightBagStream::pushMessage(
  double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data) -> void
{
  this->data_.emplace_back(time_s, deserialize(data));
}

auto TrafficLightBagStream::publishWithTimestampFix(
  TrafficLightGroupArray msg, const rclcpp::Time & ros_time) -> void
{
  const rclcpp::Time original_stamp(msg.stamp);
  msg.stamp = ros_time;
  for (auto & group : msg.traffic_light_groups) {
    for (auto & pred : group.predictions) {
      const auto diff = rclcpp::Time(pred.predicted_stamp) - original_stamp;
      pred.predicted_stamp = ros_time + diff;
    }
  }
  publisher_->publish(msg);
}

auto TrafficLightBagStream::publishUpTo(double scenario_time, const rclcpp::Time & ros_time) -> void
{
  while (index_ < data_.size() && data_[index_].first <= scenario_time) {
    publishWithTimestampFix(data_[index_].second, ros_time);
    ++index_;
  }
}

auto TrafficLightBagStream::publishNearest(double target_time_s, const rclcpp::Time & ros_time)
  -> void
{
  if (data_.empty()) {
    return;
  }
  publishWithTimestampFix(data_[nearestIndex(target_time_s)].second, ros_time);
}

}  // namespace experimental
}  // namespace simple_sensor_simulator

#endif  // PERCEPTION_REPRODUCER_HAS_TRAFFIC_LIGHT_GROUP_ARRAY
