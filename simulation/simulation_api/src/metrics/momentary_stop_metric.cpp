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
void MomentaryStopMetric::calculate()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    return;
  }
  auto id = entity_manager_ptr_->getNextStopLineId(target_entity, stop_sequence_start_distance);
  if (!id) {
    if (in_stop_sequence_) {
      THROW_METRICS_CALCULATION_ERROR("failed to find next stop line id.");
    }
    return;
  }
  if (id.get() != stop_line_lanelet_id) {
    if (in_stop_sequence_) {
      THROW_METRICS_CALCULATION_ERROR("failed to find next stop line id.");
    }
    return;
  }
  auto distance = entity_manager_ptr_->getDistanceToStopLine(
    target_entity,
    stop_sequence_start_distance);
  if (!distance) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate distance to stop line.");
  }
  if (!in_stop_sequence_ && distance.get() < stop_sequence_start_distance) {
    in_stop_sequence_ = true;
  }
  if (!in_stop_sequence_) {
    return;
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
      sequence_finished_ = true;
    }
    if (!sequence_finished_ && distance.get() <= stop_sequence_end_distance) {
      THROW_SPECIFICATION_VIOLATION_ERROR("overrun detected.");
    }
    return;
  } else {
    THROW_SPECIFICATION_VIOLATION_ERROR("acceleration is out of range.");
  }
}

bool MomentaryStopMetric::calculateFinished()
{
  return sequence_finished_;
}

nlohmann::json MomentaryStopMetric::to_json()
{
  nlohmann::json json = {{"in_stop_sequence", in_stop_sequence_}};
  json.merge_patch(MetricBase::to_base_json());
  if (!in_stop_sequence_) {
    return json;
  }
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    return json;
  }
  json.merge_patch({"linear_acceleration", status->action_status.accel.linear.x});
  auto standstill_duration = entity_manager_ptr_->getStandStillDuration(target_entity);
  if (!standstill_duration) {
    THROW_METRICS_CALCULATION_ERROR("failed to calculate standstill duration.");
  }
  json.merge_patch({"stop_duration", standstill_duration.get()});
  return json;
}
}  // namespace metrics
