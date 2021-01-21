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

#include <simulation_api/metrics/momentary_stop_metric.hpp>

#include <string>

namespace metrics
{
void MomentaryStopMetric::update()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    THROW_METRICS_CALCULATION_ERROR("failed to get target entity status.");
    return;
  }
  auto distance = entity_manager_ptr_->getDistanceToStopLine(
    target_entity,
    stop_target_lanelet_id,
    stop_sequence_start_distance);
  distance_to_stopline_ = distance.get();
  if (!distance) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate distance to stop line.");
  }
  linear_acceleration_ = status->action_status.accel.linear.x;
  if (min_acceleration <= linear_acceleration_ && linear_acceleration_ <= max_acceleration) {
    auto standstill_duration = entity_manager_ptr_->getStandStillDuration(target_entity);
    if (!standstill_duration) {
      THROW_METRICS_CALCULATION_ERROR("failed to calculate standstill duration.");
    }
    standstill_duration_ = standstill_duration.get();
    if (entity_manager_ptr_->isStopping(target_entity) &&
      standstill_duration.get() >= stop_duration)
    {
      success();
    }
    if (distance.get() <= stop_sequence_end_distance) {
      failure(SPECIFICATION_VIOLATION_ERROR("overrun detected"));
    }
    return;
  } else {
    failure(SPECIFICATION_VIOLATION_ERROR("acceleration is out of range."));
  }
}

bool MomentaryStopMetric::activateTrigger()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    return false;
  }
  auto distance = entity_manager_ptr_->getDistanceToStopLine(
    target_entity,
    stop_target_lanelet_id,
    stop_sequence_start_distance);
  if (!distance) {
    return false;
  }
  if (distance.get() <= stop_sequence_start_distance) {
    return true;
  }
  return false;
}

nlohmann::json MomentaryStopMetric::to_json()
{
  nlohmann::json json = MetricBase::to_base_json();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["linear_acceleration"] = linear_acceleration_;
    json["stop_duration"] = standstill_duration_;
    json["distance_to_stopline"] = distance_to_stopline_;
  }
  return json;
}
}  // namespace metrics
