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
#include <iostream>

namespace metrics
{
MetricsManager::MetricsManager(bool verbose) : metrics_()
{
  verbose_ = verbose;
}

void MetricsManager::setVerbose(bool verbose)
{
  verbose_ = verbose;
}

void MetricsManager::calculate()
{
  nlohmann::json log;
  std::vector<std::string> disable_metrics_list = {};
  for (auto & metric : metrics_) {
    if (metric.second->getLifecycle() == MetricLifecycle::INACTIVE) {
      if (metric.second->activateTrigger()) {
        metric.second->activate();
      }
    }
    if (metric.second->getLifecycle() == MetricLifecycle::ACTIVE) {
      metric.second->update();
    }
    log[metric.first] = metric.second->to_json();
    if (metric.second->getLifecycle() == MetricLifecycle::SUCCESS ||
      metric.second->getLifecycle() == MetricLifecycle::FAILURE)
    {
      disable_metrics_list.emplace_back(metric.first);
    }
  }
  if (verbose_) {
    for (const auto metric_json : log) {
      std::cout << metric_json << std::endl;
    }
  }
  // log_ = log;
  for (const auto name : disable_metrics_list) {
    if (metrics_[name]->getLifecycle() == MetricLifecycle::FAILURE) {
      metrics_[name]->throwException();
    }
    metrics_.erase(name);
  }
}

void MetricsManager::setEntityManager(
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr)
{
  entity_manager_ptr_ = entity_manager_ptr;
}
}  // namespace metrics
