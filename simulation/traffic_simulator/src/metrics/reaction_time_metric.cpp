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

#include <string>
#include <traffic_simulator/metrics/reaction_time_metric.hpp>

namespace metrics
{
ReactionTimeMetric::ReactionTimeMetric(
  std::string target_entity, double maximum_reaction_time, double jerk_upper_threshold,
  double jerk_lower_threshold, bool check_upper_threshold, bool check_lower_threshold)
: MetricBase("ReactionTime"),
  target_entity(target_entity),
  maximum_reaction_time(maximum_reaction_time),
  jerk_upper_threshold(jerk_upper_threshold),
  jerk_lower_threshold(jerk_lower_threshold),
  check_upper_threshold(check_upper_threshold),
  check_lower_threshold(check_lower_threshold)
{
  elapsed_duration_ = 0;
}

bool ReactionTimeMetric::activateTrigger() { return true; }

void ReactionTimeMetric::update()
{
  const auto jerk = entity_manager_ptr_->getLinearJerk(target_entity);
  current_linear_jerk_ = jerk;
  if (check_lower_threshold && jerk_lower_threshold >= jerk) {
    success();
    return;
  }
  if (check_upper_threshold && jerk_upper_threshold <= jerk) {
    success();
    return;
  }
  if (elapsed_duration_ > maximum_reaction_time) {
    failure(SPECIFICATION_VIOLATION("maximum reaction time is expired."));
    return;
  }
  elapsed_duration_ = elapsed_duration_ + entity_manager_ptr_->getStepTime();
}

nlohmann::json ReactionTimeMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["elapsed_duration"] = elapsed_duration_;
    json["current linear jerk"] = current_linear_jerk_;
  }
  return json;
}
}  // namespace metrics
