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
#include <simulation_api/metrics/metrics_base.hpp>

#include <vector>
#include <memory>
#include <utility>

namespace metrics
{
class MetricsManager
{
public:
  template<typename T, typename ... Ts>
  void addMetric(Ts && ... xs)
  {
    auto metric_ptr = new T(std::forward<Ts>(xs)...);
    metrics_.push_back(metric_ptr);
  }

  void calculate();

private:
  std::vector<MetricsBase> metrics_;
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr_;
};
}  // namespace metrics

#endif  // SIMULATION_API__METRICS__METRICS_MANAGER_HPP_
