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
#include <traffic_simulator/metrics/pull_over_metric.hpp>

namespace metrics
{
PullOverMetric::PullOverMetric(
  const std::string & target_entity, std::int64_t target_lanelet_id,
  double threshold_standstill_duration, double threshold_yaw)
: MetricBase("PullOver"),
  target_entity(target_entity),
  target_lanelet_id(target_lanelet_id),
  threshold_standstill_duration(threshold_standstill_duration),
  threshold_yaw(threshold_yaw)
{
  if (threshold_yaw < 0) {
    THROW_SEMANTIC_ERROR(
      "threshold_yaw value in PullOverMetric should be over 0, value : ", threshold_yaw);
  }
}

void PullOverMetric::update()
{
  if (entity_manager_ptr_->getStandStillDuration(target_entity) >= threshold_standstill_duration) {
    const auto lanelet_pose = entity_manager_ptr_->getLaneletPose(target_entity);
    if (lanelet_pose && std::abs(lanelet_pose->rpy.z) <= threshold_yaw) {
    } else {
      failure(SPECIFICATION_VIOLATION(
        "pulling over entity : ", target_entity,
        " was failed because of this entity stops diagonally."));
      return;
    }
  }
}

nlohmann::json PullOverMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  return json;
}

bool PullOverMetric::activateTrigger()
{
  return entity_manager_ptr_->isInLanelet(target_entity, target_lanelet_id, 1.0);
}
}  // namespace metrics
