// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__METRICS__OUT_OF_RANGE_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__OUT_OF_RANGE_METRIC_HPP_

#include <limits>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class OutOfRangeMetric : public MetricBase
{
public:
  struct Config
  {
    std::string target_entity;
    double min_velocity = -std::numeric_limits<double>::max();
    double max_velocity = std::numeric_limits<double>::max();
    double min_acceleration = -std::numeric_limits<double>::max();
    double max_acceleration = std::numeric_limits<double>::max();
    double min_jerk = -std::numeric_limits<double>::max();
    double max_jerk = std::numeric_limits<double>::max();

    boost::optional<std::string> jerk_topic = boost::none;
  };

  explicit OutOfRangeMetric(const Config & config)
  : OutOfRangeMetric(
      config.target_entity, config.min_velocity, config.max_velocity, config.min_acceleration,
      config.max_acceleration, config.min_jerk, config.max_jerk, config.jerk_topic)
  {
  }

  /**
   * @brief Construct a new Out Of Range Metric object
   * @param target_entity Name of target entity
   * @param min_velocity If the velocity of the target entity go below this value, this metrics becomes failure state.
   * @param max_velocity If the velocity of the target entity overs this value, this metrics becomes failure state.
   * @param min_acceleration If the acceleration of the target entity go below this value, this metrics becomes failure state.
   * @param max_acceleration If the acceleration of the target entity overs this value, this metrics becomes failure state.
   * @param min_jerk If the jerk of the target entity go below this value, this metrics becomes failure state.
   * @param max_jerk If the jerk of the target entity overs this value, this metrics becomes failure state.
   */
  OutOfRangeMetric(
    std::string target_entity, double min_velocity, double max_velocity, double min_acceleration,
    double max_acceleration, double min_jerk, double max_jerk,
    const boost::optional<std::string> & jerk_topic = boost::none)
  : MetricBase("MomentaryStop"),
    target_entity(std::move(target_entity)),
    min_velocity(min_velocity),
    max_velocity(max_velocity),
    min_acceleration(min_acceleration),
    max_acceleration(max_acceleration),
    min_jerk(min_jerk),
    max_jerk(max_jerk)
  {
    if (jerk_topic) {
      node_ptr_ = std::make_unique<rclcpp::Node>("momentary_stop_metrics_" + target_entity);
      jerk_callback_ptr_ = node_ptr_->create_subscription<std_msgs::msg::Float32>(
        *jerk_topic, rclcpp::QoS(1),
        [this](const std_msgs::msg::Float32::SharedPtr msg) { linear_jerk_ = msg->data; });
    }
  }

  void update() override;
  bool activateTrigger() override { return true; }
  nlohmann::json toJson() override;

  const std::string target_entity;
  const double min_velocity;
  const double max_velocity;
  const double min_acceleration;
  const double max_acceleration;
  const double min_jerk;
  const double max_jerk;
  const boost::optional<std::string> jerk_topic;

private:
  double linear_velocity_ = 0;
  double linear_acceleration_ = 0;
  double linear_jerk_ = 0;

  std::unique_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr jerk_callback_ptr_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__OUT_OF_RANGE_METRIC_HPP_
