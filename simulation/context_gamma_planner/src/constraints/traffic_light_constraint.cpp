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

#include <context_gamma_planner/constraints/traffic_light_constraint.hpp>

namespace context_gamma_planner
{
namespace constraints
{
TrafficLightConstraint::TrafficLightConstraint(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Id traffic_light_id,
  const lanelet::Id stop_line_id)
: ConstraintBase(
    hdmap_utils, TRAFFIC_LIGHT_CONSTRAINT,
    {traffic_simulator::lanelet_wrapper::lanelet_map::stopLinePolygon(stop_line_id)}),
  traffic_light_id(traffic_light_id),
  stop_line_id(stop_line_id)
{
}
}  // namespace constraints
}  // namespace context_gamma_planner
