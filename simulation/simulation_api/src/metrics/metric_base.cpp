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

#include <simulation_api/metrics/metric_base.hpp>

#include <string>
#include <memory>

namespace metrics
{
MetricBase::MetricBase(std::string target_entity, std::string metrics_type)
: target_entity(target_entity), metrics_type(metrics_type) {}

void MetricBase::foundSpecificationViolation(std::string message)
{
  message = "target_entity : " + target_entity + "\n" +
    "metrics_type : " + metrics_type + "\n" + message;
  throw SpecificationViolationError(message);
}

void MetricBase::setEntityManager(
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr)
{
  entity_manager_ptr_ = entity_manager_ptr;
}
}  // namespace metrics
