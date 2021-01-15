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

#include <simulation_api/metrics/traveled_distance_metric.hpp>

#include <string>

namespace metrics
{
TraveledDistanceMetric::TraveledDistanceMetric(std::string target_entity, double step_time)
: MetricBase(target_entity, "TraveledDistance"), step_time(step_time)
{
  traveled_distance = 0;
}

void TraveledDistanceMetric::calculate()
{
  auto status = entity_manager_ptr_->getEntityStatus(target_entity);
  if (status) {
    traveled_distance = traveled_distance + status.get().action_status.twist.linear.x * step_time;
  }
}
}  // namespace metrics
