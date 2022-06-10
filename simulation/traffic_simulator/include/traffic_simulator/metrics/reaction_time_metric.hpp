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

#ifndef TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_

#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class ReactionTimeMetric : public MetricBase
{
public:
  /**
   * @brief Construct a new Reaction Time Metric object
   * @param target_entity name of the target entity
   * @param maximum_reaction_time maximum time
   * @param jerk_upper_threshold If check_upper_threshold = true and the jerk of target entity overs this value, the metric becomes failure state.
   * @param jerk_lower_threshold If check_lower_threshold = true the jerk of target entity go below this value, the metric becomes failure state.
   * @param check_upper_threshold If true, check upper threshold of the jerk.
   * @param check_lower_threshold If true, check lower threshold of the jerk.
   */
  explicit ReactionTimeMetric(
    std::string target_entity, double maximum_reaction_time, double jerk_upper_threshold,
    double jerk_lower_threshold, bool check_upper_threshold = true,
    bool check_lower_threshold = true);
  ~ReactionTimeMetric() override = default;
  void update() override;
  nlohmann::json toJson();
  bool activateTrigger() override;
  const std::string target_entity;
  const double maximum_reaction_time;
  const double jerk_upper_threshold;
  const double jerk_lower_threshold;
  const bool check_upper_threshold;
  const bool check_lower_threshold;

private:
  double elapsed_duration_;
  double current_linear_jerk_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_
