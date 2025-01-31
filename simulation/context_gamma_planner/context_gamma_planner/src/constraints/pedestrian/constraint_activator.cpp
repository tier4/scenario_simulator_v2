// Copyright 2021 Tier IV, Inc All rights reserved.
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

#include <algorithm>
#include <context_gamma_planner/constraints/pedestrian/constraint_activator.hpp>

namespace context_gamma_planner
{
namespace pedestrian
{
namespace constraints
{
using State = context_gamma_planner::constraints::State;

ConstraintActivator::ConstraintActivator(
  const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
  const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr)
: context_gamma_planner::constraints::ConstraintActivatorBase(hd_map_utils_ptr, traffic_lights_ptr)
{
}

}  // namespace constraints
}  // namespace pedestrian
}  // namespace context_gamma_planner
