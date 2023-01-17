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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__REACTION_TIME_MONITOR_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__REACTION_TIME_MONITOR_HPP_

#include <optional>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>

namespace traffic_simulator::entity
{
class ReactionTimeMonitor
{
public:
  /**
   * @brief Construct a new Reaction Time Monitor object
   * @param entity name of the target entity
   * @param max_reaction_time maximum time
   * @param jerk_upper_threshold If not std::nullopt and the jerk of target entity exceeds this value, this monitor throws SPECIFICATION_VIOLATION.
   * @param jerk_lower_threshold If not std::nullopt and the jerk of target entity falls below this value, this monitor throws SPECIFICATION_VIOLATION.
   */
  explicit ReactionTimeMonitor(
    EntityBase & entity, double max_reaction_time, std::optional<double> upper_jerk_threshold,
    std::optional<double> lower_jerk_threshold);

  auto operator()(double) -> bool;

private:
  const double max_reaction_time_;
  const std::optional<double> upper_jerk_threshold_;
  const std::optional<double> lower_jerk_threshold_;

  EntityBase & entity_;
  double elapsed_duration_ = 0.0;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__REACTION_TIME_MONITOR_HPP_
