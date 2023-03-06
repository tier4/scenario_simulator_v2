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
#include <traffic_simulator/entity/monitor/reaction_time_monitor.hpp>

#include "traffic_simulator/entity/entity_base.hpp"

namespace traffic_simulator::entity
{
ReactionTimeMonitor::ReactionTimeMonitor(
  const EntityBase & entity, double max_reaction_time, std::optional<double> upper_jerk_threshold,
  std::optional<double> lower_jerk_threshold)
: max_reaction_time_(max_reaction_time),
  upper_jerk_threshold_(upper_jerk_threshold),
  lower_jerk_threshold_(lower_jerk_threshold),
  entity_(entity)
{
}

auto ReactionTimeMonitor::operator()(double step_time) -> bool
{
  const auto jerk = entity_.getLinearJerk();
  if (lower_jerk_threshold_ && *lower_jerk_threshold_ >= jerk) {
    return true;
  }
  if (upper_jerk_threshold_ && *upper_jerk_threshold_ <= jerk) {
    return true;
  }
  if (elapsed_duration_ > max_reaction_time_) {
    throw SPECIFICATION_VIOLATION("maximum reaction time exceeded.");
  }
  elapsed_duration_ = elapsed_duration_ + step_time;
  return false;
}

}  // namespace traffic_simulator::entity
