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

#include <iostream>
#include <memory>
#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>
#include <traffic_simulator/metrics/metrics_manager.hpp>
#include <vector>

namespace metrics
{
MetricsManager::MetricsManager(
  const boost::filesystem::path & log_path, const bool verbose, const bool write_file_every_frame)
: log_path(log_path),
  write_file_every_frame(write_file_every_frame),
  verbose_(verbose),
  metrics_(),
  file_(log_path.string())
{
}

bool MetricsManager::exists(const std::string & name) const
{
  if (metrics_.find(name) == metrics_.end()) {
    return false;
  }
  return true;
}

void MetricsManager::setVerbose(const bool verbose) { verbose_ = verbose; }

MetricLifecycle MetricsManager::getLifecycle(const std::string & name)
{
  if (!exists(name)) {
    THROW_SEMANTIC_ERROR("metrics name : ", name, " does not exist.");
  }
  return metrics_.at(name)->getLifecycle();
}

void MetricsManager::calculate()
{
  nlohmann::json log;
  std::vector<std::string> disable_metrics_list = {};
  for (const auto & metric : metrics_) {
    if (metric.second->getLifecycle() == MetricLifecycle::INACTIVE) {
      if (metric.second->activateTrigger()) {
        metric.second->activate();
      }
    }
    if (metric.second->getLifecycle() == MetricLifecycle::ACTIVE) {
      metric.second->update();
    }
    log[metric.first] = metric.second->toJson();
    if (verbose_) {
      std::cout << "metric : " << metric.first << " => " << log[metric.first] << std::endl;
    }
    if (
      metric.second->getLifecycle() == MetricLifecycle::SUCCESS ||
      metric.second->getLifecycle() == MetricLifecycle::FAILURE) {
      disable_metrics_list.emplace_back(metric.first);
    }
  }
  for (const auto & name : disable_metrics_list) {
    if (metrics_[name]->getLifecycle() == MetricLifecycle::FAILURE) {
      metrics_[name]->throwException();
    }
    // metrics_.erase(name);
  }
  log_[std::to_string(entity_manager_ptr_->getCurrentTime())] = log;
  file_ << log_;
}

void MetricsManager::setEntityManager(
  const std::shared_ptr<traffic_simulator::entity::EntityManager> & entity_manager_ptr)
{
  entity_manager_ptr_ = entity_manager_ptr;
}
}  // namespace metrics
