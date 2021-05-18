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

#ifndef TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_
#define TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_

#include <boost/optional.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>

namespace metrics
{
class SpecificationViolationError : public std::runtime_error
{
public:
  explicit SpecificationViolationError(const char * message) : runtime_error(message) {}
  explicit SpecificationViolationError(std::string message) : runtime_error(message.c_str()) {}
  explicit SpecificationViolationError(std::string message, const char * file, int line)
  : runtime_error(message + "\nFile:" + file + "\nLine:" + std::to_string(line))
  {
  }
};

#define SPECIFICATION_VIOLATION_ERROR(description) \
  SpecificationViolationError(description, __FILE__, __LINE__)

enum class MetricLifecycle { INACTIVE, ACTIVE, FAILURE, SUCCESS };

class MetricBase
{
public:
  explicit MetricBase(std::string metrics_type);
  virtual ~MetricBase();
  virtual bool activateTrigger() = 0;
  virtual void update() = 0;
  void success();
  void failure(SpecificationViolationError error);
  void activate();
  virtual nlohmann::json to_json() = 0;
  nlohmann::json to_base_json();
  void setEntityManager(
    std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr);
  const std::string metrics_type;
  MetricLifecycle getLifecycle() { return lifecycle_; }
  void throwException();

protected:
  std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr_;

private:
  boost::optional<SpecificationViolationError> error_;
  MetricLifecycle lifecycle_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__METRIC_BASE_HPP_
