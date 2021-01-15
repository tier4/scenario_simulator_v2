// Copyright 2015-2021 TierIV.inc. All rights reserved.
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

#include <simulation_api/metrics/metrics_manager.hpp>
#include <simulation_api/metrics/metrics_base.hpp>

namespace metrics
{
void MetricsManager::addMetrics(MetricsBase metrics)
{
  metrics_list_.emplace_back(metrics);
}

void MetricsManager::calculate()
{
  for (auto & metrics : metrics_list_) {
    metrics.calculate();
  }
}
}  // namespace metrics
