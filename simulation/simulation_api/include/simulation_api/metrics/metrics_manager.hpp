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

#ifndef SIMULATION_API__METRICS__METRICS_MANAGER_HPP_
#define SIMULATION_API__METRICS__METRICS_MANAGER_HPP_

#include <simulation_api/entity/entity_manager.hpp>
#include <simulation_api/metrics/metric_base.hpp>

#include <nlohmann/json.hpp>

#include <unordered_map>
#include <memory>
#include <string>
#include <utility>
#include <fstream>

namespace metrics
{
class MetricsManager
{
public:
  explicit MetricsManager(bool verbose, const std::string & logfile_path);
  ~MetricsManager()
  {
    std::ofstream file(logfile_path);
    file << log_;
  }
  void setVerbose(bool verbose);
  void setEntityManager(std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr);
  template<typename T, typename ... Ts>
  void addMetric(std::string name, Ts && ... xs)
  {
    auto metric_ptr = std::make_shared<T>(std::forward<Ts>(xs)...);
    metric_ptr->setEntityManager(this->entity_manager_ptr_);
    metrics_.insert({name, metric_ptr});
  }
  void calculate();
  const std::string logfile_path;

private:
  bool verbose_;
  nlohmann::json log_;
  std::unordered_map<std::string, std::shared_ptr<MetricBase>> metrics_;
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr_;
};
}  // namespace metrics

#endif  // SIMULATION_API__METRICS__METRICS_MANAGER_HPP_
