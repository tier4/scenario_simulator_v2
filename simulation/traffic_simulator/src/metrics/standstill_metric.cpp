// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <string>
#include <traffic_simulator/metrics/standstill_metric.hpp>

namespace metrics
{
StandstillMetric::StandstillMetric(std::string target_entity, double allow_standstill_duration)
: MetricBase("StandStillMetric"),
  target_entity(target_entity),
  allow_standstill_duration(allow_standstill_duration)
{
  standstill_duration_ = boost::none;
}

bool StandstillMetric::activateTrigger() { return true; }

void StandstillMetric::update()
{
  standstill_duration_ = entity_manager_ptr_->getStandStillDuration(target_entity);
  if (standstill_duration_ && standstill_duration_.get() >= allow_standstill_duration) {
    failure(SPECIFICATION_VIOLATION(
      "Standstill duration over ", allow_standstill_duration, " seconds. Stand still duration is ",
      standstill_duration_.get()));
    return;
  }
}

nlohmann::json StandstillMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  if (getLifecycle() == MetricLifecycle::ACTIVE) {
    if (standstill_duration_) {
      json["standstill_duration"] = standstill_duration_.get();
    } else {
      json["standstill_duration"] = "none";
    }
  }
  return json;
}
}  // namespace metrics
