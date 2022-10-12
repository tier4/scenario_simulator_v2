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

#include <optional>
#include <string>
#include <traffic_simulator/metrics/momentary_stop_metric.hpp>

namespace metrics
{
void MomentaryStopMetric::update()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    THROW_SIMULATION_ERROR("failed to get target entity status.");
    return;
  }
  std::optional<double> distance;
  switch (stop_target_lanelet_type) {
    case StopTargetLaneletType::STOP_LINE:
      distance = entity_manager_ptr_->getDistanceToStopLine(target_entity, stop_target_lanelet_id);
      break;
    case StopTargetLaneletType::CROSSWALK:
      distance = entity_manager_ptr_->getDistanceToCrosswalk(target_entity, stop_target_lanelet_id);
      break;
    default:
      THROW_SIMULATION_ERROR("invalid lanelet type.");
      break;
  }
  if (!distance) {
    THROW_SIMULATION_ERROR("failed to calculate distance to stop line.");
  }
  distance_to_stopline_ = distance.value();
  linear_acceleration_ = status->action_status.accel.linear.x;
  if (min_acceleration <= linear_acceleration_ && linear_acceleration_ <= max_acceleration) {
    if (standstill_duration_ = entity_manager_ptr_->getStandStillDuration(target_entity);
        entity_manager_ptr_->isStopping(target_entity) && standstill_duration_ >= stop_duration) {
      success();
    }
    if (distance.value() <= stop_sequence_end_distance) {
      failure(SPECIFICATION_VIOLATION("overrun detected"));
    }
    return;
  } else {
    failure(SPECIFICATION_VIOLATION("acceleration is out of range"));
  }
}

bool MomentaryStopMetric::activateTrigger()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (!status) {
    return false;
  }
  std::optional<double> distance;
  switch (stop_target_lanelet_type) {
    case StopTargetLaneletType::STOP_LINE:
      distance = entity_manager_ptr_->getDistanceToStopLine(target_entity, stop_target_lanelet_id);
      break;
    case StopTargetLaneletType::CROSSWALK:
      distance = entity_manager_ptr_->getDistanceToCrosswalk(target_entity, stop_target_lanelet_id);
      break;
    default:
      THROW_SIMULATION_ERROR("invalid lanelet type.");
      break;
  }
  if (!distance) {
    return false;
  }
  if (distance.value() <= stop_sequence_start_distance) {
    return true;
  }
  return false;
}

nlohmann::json MomentaryStopMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["linear_acceleration"] = linear_acceleration_;
    json["stop_duration"] = standstill_duration_;
    json["distance_to_stopline"] = distance_to_stopline_;
  }
  return json;
}
}  // namespace metrics
