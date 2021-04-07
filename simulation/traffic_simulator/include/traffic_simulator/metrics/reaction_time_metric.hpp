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

#ifndef TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_

#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class ReactionTimeMetric : public MetricBase
{
public:
  explicit ReactionTimeMetric(
    std::string target_entity, double maximum_reaction_time, double jerk_upper_threashold,
    double jerk_lower_threashold, bool check_upper_threashold = true,
    bool check_lower_threashold = true);
  ~ReactionTimeMetric() override = default;
  void update() override;
  nlohmann::json to_json();
  bool activateTrigger() override;
  const std::string target_entity;
  const double maximum_reaction_time;
  const double jerk_upper_threashold;
  const double jerk_lower_threashold;
  const bool check_upper_threashold;
  const bool check_lower_threashold;

private:
  double elapsed_duration_;
  double current_linear_jerk_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__REACTION_TIME_METRIC_HPP_
