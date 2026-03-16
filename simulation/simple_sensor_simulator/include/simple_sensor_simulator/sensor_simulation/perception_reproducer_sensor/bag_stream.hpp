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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__BAG_STREAM_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__BAG_STREAM_HPP_

#include <rosbag2_storage/serialized_bag_message.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{

template <typename T>
class BagStreamBase
{
public:
  explicit BagStreamBase(const std::string & topic_name) : topic_name_(topic_name) {}

  virtual ~BagStreamBase() = default;

  auto tryPushMessage(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message,
    double time_s) -> void
  {
    if (bag_message->topic_name == topic_name_) {
      pushMessage(time_s, bag_message->serialized_data);
    }
  }

  auto empty() const -> bool { return data_.empty(); }

  static auto deserialize(const std::shared_ptr<rcutils_uint8_array_t> & data) -> T
  {
    rclcpp::SerializedMessage serialized_message(*data);
    T message;
    rclcpp::Serialization<T> serialization;
    serialization.deserialize_message(&serialized_message, &message);
    return message;
  }

protected:
  virtual auto pushMessage(
    double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data) -> void = 0;

  const std::string topic_name_;

  std::vector<std::pair<double, T>> data_;
};

template <typename T>
class BagStream : public BagStreamBase<T>
{
public:
  BagStream(const std::string & topic_name, typename rclcpp::Publisher<T>::SharedPtr publisher)
  : BagStreamBase<T>(topic_name), publisher_(publisher)
  {
  }

  auto publishUpTo(double scenario_time, const rclcpp::Time & ros_time) -> void
  {
    while (index_ < this->data_.size() && this->data_[index_].first <= scenario_time) {
      auto & message = this->data_[index_].second;
      message.header.stamp = ros_time;
      publisher_->publish(message);
      ++index_;
    }
  }

  auto reset() -> void { index_ = 0; }

  auto done() const -> bool { return index_ >= this->data_.size(); }

protected:
  auto pushMessage(
    double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data) -> void override;

private:
  typename rclcpp::Publisher<T>::SharedPtr publisher_;

  size_t index_ = 0;
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__BAG_STREAM_HPP_
