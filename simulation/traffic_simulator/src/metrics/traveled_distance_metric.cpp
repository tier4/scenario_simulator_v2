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

#include <string>
#include <traffic_simulator/metrics/traveled_distance_metric.hpp>

namespace metrics
{
TraveledDistanceMetric::TraveledDistanceMetric(std::string target_entity)
: MetricBase("TraveledDistance"), target_entity(target_entity)
{
  traveled_distance = 0;
}

bool TraveledDistanceMetric::activateTrigger() { return true; }

void TraveledDistanceMetric::update()
{
  double step_time = entity_manager_ptr_->getStepTime();
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (status) {
    traveled_distance =
      traveled_distance + std::fabs(status.get().action_status.twist.linear.x) * step_time;
  }
}

nlohmann::json TraveledDistanceMetric::to_json()
{
  nlohmann::json json = MetricBase::to_base_json();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["traveled_distance"] = traveled_distance;
  }
  return json;
}
}  // namespace metrics
