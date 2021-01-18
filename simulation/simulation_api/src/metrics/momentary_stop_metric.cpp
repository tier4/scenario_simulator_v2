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
}

bool MomentaryStopMetric::calculateFinished()
{
  return false;
}
}  // namespace metrics
