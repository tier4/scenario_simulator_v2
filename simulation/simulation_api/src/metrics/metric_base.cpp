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
MetricBase::MetricBase(std::string metrics_type)
: metrics_type(metrics_type)
{
  lifecycle_ = MetricLifecycle::INACTIVE;
  error_ = boost::none;
}

MetricBase::~MetricBase() {}

void MetricBase::setEntityManager(
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr)
{
  entity_manager_ptr_ = entity_manager_ptr;
}

void MetricBase::success()
{
  if (lifecycle_ != MetricLifecycle::ACTIVE) {
    THROW_METRICS_CALCULATION_ERROR("lifecycle of the metric should be active");
  }
  lifecycle_ = MetricLifecycle::SUCCESS;
}

void MetricBase::activate()
{
  if (lifecycle_ != MetricLifecycle::INACTIVE) {
    THROW_METRICS_CALCULATION_ERROR("lifecycle of the metric should be inactive");
  }
  lifecycle_ = MetricLifecycle::ACTIVE;
}

void MetricBase::failure(SpecificationViolationError error)
{
  if (lifecycle_ != MetricLifecycle::ACTIVE) {
    THROW_METRICS_CALCULATION_ERROR("lifecycle of the metric should be active");
  }
  error_ = error;
  lifecycle_ = MetricLifecycle::FAILURE;
}

nlohmann::json MetricBase::to_base_json()
{
  nlohmann::json json;
  json["type"] = metrics_type;
  std::string lifecycle;
  switch (lifecycle_) {
    case MetricLifecycle::INACTIVE:
      lifecycle = "inactive";
      break;
    case MetricLifecycle::ACTIVE:
      lifecycle = "active";
      break;
    case MetricLifecycle::FAILURE:
      lifecycle = "failure";
      break;
    case MetricLifecycle::SUCCESS:
      lifecycle = "success";
      break;
  }
  json["lifecycle"] = lifecycle;
  return json;
}

void MetricBase::throwException()
{
  if (error_) {
    throw error_.get();
  }
  THROW_METRICS_CALCULATION_ERROR("error is empty");
}
}  // namespace metrics
