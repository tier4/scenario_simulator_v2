// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/metrics/collision_metric.hpp>

namespace metrics
{
CollisionMetric::CollisionMetric(std::string target_entity)
: MetricBase("Collision"),
  target_entity(target_entity),
  check_targets_({}),
  check_collision_with_all_entities_(true)
{
}

CollisionMetric::CollisionMetric(
  std::string target_entity, const std::vector<std::string> & check_targets)
: MetricBase("Collision"),
  target_entity(target_entity),
  check_targets_(check_targets),
  check_collision_with_all_entities_(false)
{
}

bool CollisionMetric::activateTrigger() { return true; }

void CollisionMetric::update()
{
  std::vector<std::string> check_targets;
  if (check_collision_with_all_entities_) {
    check_targets = entity_manager_ptr_->getEntityNames();
  } else {
    check_targets = check_targets_;
  }
  for (const auto & entity_name : check_targets) {
    if (entity_manager_ptr_->checkCollision(target_entity, entity_name)) {
      failure(SPECIFICATION_VIOLATION(
        "Collision detected, entity : ", target_entity, "and entity : ", entity_name,
        " was collided."));
      return;
    }
  }
}

nlohmann::json CollisionMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
  }
  return json;
}
}  // namespace metrics
