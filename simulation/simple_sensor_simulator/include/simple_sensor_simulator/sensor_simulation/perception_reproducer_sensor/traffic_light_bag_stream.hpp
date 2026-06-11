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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__TRAFFIC_LIGHT_BAG_STREAM_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__TRAFFIC_LIGHT_BAG_STREAM_HPP_

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#define PERCEPTION_REPRODUCER_HAS_TRAFFIC_LIGHT_GROUP_ARRAY

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/bag_stream.hpp>

namespace simple_sensor_simulator
{
inline namespace experimental
{

class TrafficLightBagStream
: public BagStreamBase<autoware_perception_msgs::msg::TrafficLightGroupArray>
{
  using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;

public:
  TrafficLightBagStream(
    const std::string & topic_name, rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr publisher);

  auto publishUpTo(double scenario_time, const rclcpp::Time & ros_time) -> void;

  auto publishNearest(double target_time_s, const rclcpp::Time & ros_time) -> void;

  auto reset() -> void { index_ = 0; }

  auto done() const -> bool { return index_ >= data_.size(); }

protected:
  auto pushMessage(double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data)
    -> void override;

private:
  auto publishWithTimestampFix(TrafficLightGroupArray msg, const rclcpp::Time & ros_time) -> void;

  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr publisher_;
  size_t index_ = 0;
};

}  // namespace experimental
}  // namespace simple_sensor_simulator

#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__TRAFFIC_LIGHT_BAG_STREAM_HPP_
