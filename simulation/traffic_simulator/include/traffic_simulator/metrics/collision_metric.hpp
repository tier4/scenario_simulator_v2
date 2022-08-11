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

#ifndef TRAFFIC_SIMULATOR__METRICS__COLLISION_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__COLLISION_METRIC_HPP_

#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class CollisionMetric : public MetricBase
{
public:
  /**
   * @brief Construct a new Traveled Distance Metric object
   * If you call this constructor, collision metric check collision with all entities.
   * 
   * @param target_entity name of the target entity
   */
  explicit CollisionMetric(std::string target_entity);
  /**
   * @brief Construct a new Collision Metric object
   * If your call this constructor, collision metric check collision with check target entities.
   * 
   * @param target_entity name of the target entity
   * @param check_targets targets which you want to check collision
   */
  explicit CollisionMetric(
    std::string target_entity, const std::vector<std::string> & check_targets);
  ~CollisionMetric() override = default;
  void update() override;
  nlohmann::json toJson();
  bool activateTrigger() override;
  const std::string target_entity;

private:
  const std::vector<std::string> check_targets_;
  const bool check_collision_with_all_entities_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__COLLISION_METRIC_HPP_
