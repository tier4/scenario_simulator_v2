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

#ifndef SIMULATION_API__METRICS__METRICS_BASE_HPP_
#define SIMULATION_API__METRICS__METRICS_BASE_HPP_

#include <simulation_api/entity/entity_manager.hpp>

#include <stdexcept>
#include <string>
#include <memory>

namespace metrics
{
class SpecificationViolationError : public std::runtime_error
{
public:
  explicit SpecificationViolationError(const char * message)
  : runtime_error(message) {}
  explicit SpecificationViolationError(std::string message)
  : runtime_error(message.c_str()) {}

private:
};

class MetricsBase
{
public:
  MetricsBase(std::string target_entity, std::string metrics_type);
  void calculate() {}
  void setEntityManager(std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr);
  const std::string target_entity;
  const std::string metrics_type;

protected:
  void foundSpecificationViolation(std::string message);

private:
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr_;
};
}  // namespace metrics

#endif  // SIMULATION_API__METRICS__METRICS_BASE_HPP_
