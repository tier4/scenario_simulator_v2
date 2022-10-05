// Copyright 2015 TIER IV.inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_
#define TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_

#include <optional>
#include <memory>
#include <nlohmann/json.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>

namespace metrics
{
enum class MetricLifecycle { INACTIVE, ACTIVE, FAILURE, SUCCESS };

class MetricBase
{
public:
  explicit MetricBase(std::string metrics_type);
  virtual ~MetricBase();
  virtual bool activateTrigger() = 0;
  virtual void update() = 0;
  void success();
  void failure(const common::scenario_simulator_exception::SpecificationViolation & error);
  void activate();
  virtual nlohmann::json toJson() = 0;
  nlohmann::json toBaseJson();
  virtual void setEntityManager(
    std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr);
  const std::string metrics_type;
  MetricLifecycle getLifecycle() { return lifecycle_; }
  void throwException();

protected:
  std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr_;

private:
  std::optional<common::scenario_simulator_exception::SpecificationViolation> error_;
  MetricLifecycle lifecycle_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_
