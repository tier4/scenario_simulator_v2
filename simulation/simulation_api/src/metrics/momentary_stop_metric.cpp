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
  auto id = entity_manager_ptr_->getNextStopLineId(target_entity, stop_sequence_start_distance);
  if (!id) {
    THROW_METRICS_CALCULATION_ERROR("failed to find next stop line id.");
    return;
  }
  if (id.get() != stop_line_lanelet_id) {
    THROW_METRICS_CALCULATION_ERROR("failed to find next stop line id.");
    return;
  }
  auto distance = entity_manager_ptr_->getDistanceToStopLine(
    target_entity,
    stop_sequence_start_distance);
  if (!distance) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate distance to stop line.");
  }
  if (min_acceleration <= status->action_status.accel.linear.x &&
    status->action_status.accel.linear.x <= max_acceleration)
  {
    auto standstill_duration = entity_manager_ptr_->getStandStillDuration(target_entity);
    if (!standstill_duration) {
      THROW_METRICS_CALCULATION_ERROR("failed to calculate standstill duration.");
    }
    if (entity_manager_ptr_->isStopping(target_entity) &&
      standstill_duration.get() > stop_duration)
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
  auto id = entity_manager_ptr_->getNextStopLineId(target_entity, stop_sequence_start_distance);
  if (!id) {
    return false;
  }
  if (id.get() != stop_line_lanelet_id) {
    return false;
  }
  auto distance = entity_manager_ptr_->getDistanceToStopLine(
    target_entity,
    stop_sequence_start_distance);
  if (!distance) {
    return false;
  }
  if (distance.get() <= stop_sequence_start_distance) {
    return true;
  }
  return false;
}

bool MomentaryStopMetric::calculateFinished()
{
  // return sequence_finished_;
  return true;
}

nlohmann::json MomentaryStopMetric::to_json()
{
  nlohmann::json json = {};
  json.merge_patch(MetricBase::to_base_json());
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    return json;
  }
  json["linear_acceleration"] = status->action_status.accel.linear.x;
  auto standstill_duration = entity_manager_ptr_->getStandStillDuration(target_entity);
  if (!standstill_duration) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate standstill duration.");
  }
  json["stop_duration"] = standstill_duration.get();
  return json;
}
}  // namespace metrics
