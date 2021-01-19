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
#include <simulation_api/metrics/metric_base.hpp>

#include <memory>
#include <vector>
#include <string>

namespace metrics
{
void MetricsManager::calculate()
{
  nlohmann::json log;
  std::vector<std::string> disable_metrics_list = {};
  for (auto & metric : metrics_) {
    metric.second->calculate();
    const auto json = metric.second->to_json();
    log.merge_patch({metric.first, json});
    if (!metric.second->calculateFinished()) {
      disable_metrics_list.emplace_back(metric.first);
    }
  }
  log_ = log;
  for (const auto name : disable_metrics_list) {
    metrics_.erase(name);
  }
}

nlohmann::json MetricsManager::getJsonLog()
{
  return log_;
}

void MetricsManager::setEntityManager(
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr)
{
  entity_manager_ptr_ = entity_manager_ptr;
}
}  // namespace metrics
