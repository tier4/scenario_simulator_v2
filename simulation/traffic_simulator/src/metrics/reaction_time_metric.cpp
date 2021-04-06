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

#include <traffic_simulator/metrics/reaction_time_metric.hpp>

#include <string>

namespace metrics
{
ReactionTimeMetric::ReactionTimeMetric(
  std::string target_entity,
  double maximum_reaction_time,
  double jerk_upper_threashold,
  double jerk_lower_threashold,
  bool check_upper_threashold,
  bool check_lower_threashold)
: MetricBase("ReactionTime"),
  target_entity(target_entity),
  maximum_reaction_time(maximum_reaction_time),
  jerk_upper_threashold(jerk_upper_threashold),
  jerk_lower_threashold(jerk_lower_threashold),
  check_upper_threashold(check_upper_threashold),
  check_lower_threashold(check_lower_threashold)
{
  elapsed_duration_ = 0;
}

bool ReactionTimeMetric::activateTrigger()
{
  return true;
}

void ReactionTimeMetric::update()
{
  const auto jerk = entity_manager_ptr_->getLinearJerk(target_entity);
  if (!jerk) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate linear jerk.");
  }
  current_linear_jerk_ = jerk.get();
  if (check_lower_threashold && jerk_lower_threashold >= jerk.get()) {
    success();
    return;
  }
  if (check_upper_threashold && jerk_upper_threashold <= jerk.get()) {
    success();
    return;
  }
  if (elapsed_duration_ > maximum_reaction_time) {
    failure(SPECIFICATION_VIOLATION_ERROR("maximum reaction time is expired."));
    return;
  }
  elapsed_duration_ = elapsed_duration_ + entity_manager_ptr_->getStepTime();
}

nlohmann::json ReactionTimeMetric::to_json()
{
  nlohmann::json json = MetricBase::to_base_json();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["elapsed_duration"] = elapsed_duration_;
    json["current linear jerk"] = current_linear_jerk_;
  }
  return json;
}
}  // namespace metrics
