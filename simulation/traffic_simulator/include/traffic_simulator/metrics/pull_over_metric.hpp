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

#ifndef TRAFFIC_SIMULATOR__METRICS__OUT_OF_RANGE_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__OUT_OF_RANGE_METRIC_HPP_

#include <limits>
#include <rclcpp/qos.hpp>
#include <string>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class PullOverMetric : public MetricBase
{
public:
  PullOverMetric(
    const std::string & target_entity, std::int64_t target_lanelet_id,
    double threshold_standstill_duration, double threshold_yaw, double threshold_lateral_distance);
  ~PullOverMetric() override = default;
  void update() override;
  nlohmann::json toJson();
  bool activateTrigger() override;
  const std::string target_entity;
  const std::int64_t target_lanelet_id;
  const double threshold_standstill_duration;
  const double threshold_yaw;
  const double threshold_lateral_distance;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__PULL_OVER_METRIC_HPP_
