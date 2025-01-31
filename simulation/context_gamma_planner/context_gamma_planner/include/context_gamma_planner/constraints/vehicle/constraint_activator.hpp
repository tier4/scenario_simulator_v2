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

#ifndef CONTEXT_GAMMA_PLANNER__VEHICLE__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
#define CONTEXT_GAMMA_PLANNER__VEHICLE__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_

#include <customized_rvo2/RVO.h>

#include <context_gamma_planner/constraints/constraint_activator_base.hpp>
#include <geometry/spline/catmull_rom_subspline.hpp>
#include <memory>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>

namespace context_gamma_planner
{
namespace vehicle
{
namespace constraints
{
class ConstraintActivator : public context_gamma_planner::constraints::ConstraintActivatorBase
{
public:
  ConstraintActivator(
    const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
    const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr);
  /// @param stop_velocity_threshold [m/s]  Set a small value since a complete stop is not possible.
  void appendStopLineConstraint(
    const lanelet::Ids & route_ids,
    const std::shared_ptr<math::geometry::CatmullRomSpline> reference_trajectory,
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & entity_status,
    const double stop_line_enable_threshold = 4.0,
    const double stop_velocity_threshold = 0.1) override;
  void appendTrafficLightConstraint(const lanelet::Ids & route_ids) override;
  void appendLaneChangeConstraint(
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & entity_status,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter) override;
  void appendRoadEndConstraint(const lanelet::Ids & route_ids) override;
};
}  // namespace constraints
}  // namespace vehicle
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__VEHICLE__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
